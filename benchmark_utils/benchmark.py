import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import os
import subprocess
import json
import zipfile
import argparse
import yaml
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from tf2_geometry_msgs import do_transform_pose_stamped, do_transform_pose
from geometry_msgs.msg import PoseStamped, Pose
from nav_msgs.msg import Odometry

# Global variables

verbose = False


def parse_args():
    parser = argparse.ArgumentParser(description="Benchmark a ROS2 simulation run")
    parser.add_argument(
        "run_path", help="Path to the run folder (should contain a 'rosbags' subfolder)"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        help="Enable verbose output",
    )
    return parser.parse_args()


def read_rosbag2(file_path):
    """return a list of all the messages, in the form (topic, msg, timestamp)"""
    reader = rosbag2_py.SequentialReader()  # Initialize the reader
    reader.open(
        rosbag2_py.StorageOptions(uri=file_path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr", output_serialization_format="cdr"
        ),
    )

    topic_types = reader.get_all_topics_and_types()

    def typename(topic_name):
        for topic_type in topic_types:
            if topic_type.name == topic_name:
                return topic_type.type
        raise ValueError(f"topic {topic_name} not in bag")

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        msg_type = get_message(typename(topic))
        msg = deserialize_message(data, msg_type)
        yield topic, msg, timestamp
    del reader


def evo_metrics(
    run_path, ground_truth_topic, estimated_pose_topic, align_origin=False, plot=False
):
    """
    Aims to compute the difference between the ground_truth (provided by the simulation) and the estimated pose post-SLAM
    This is a good indicator of the 'quality of the map'
    Currently there is a problem because the ground_truth starts at (0, 0) instead of the initial poisiton of the robot
    defined in the world file. This results in the ground_truth not "fitting" into the map image. --align_origin is useful to
    aligh the odom with the ground truth, but then nothing fits into the map image.
    A solution is to edit the world file so that the initial position of the robot is at (0, 0), and we adjust the map position instead
    of the robot position.
    """
    # We use evo to compute APE (and RPE)
    # Easiest way I found is to call the bash command directly

    # Find the most recent file in the 'maps' directory
    maps_path = os.path.join(run_path, "maps")

    if plot:  # We want to hve a map to plot the results
        latest_map_file = None
        if os.path.isdir(maps_path):
            map_files = [
                os.path.join(maps_path, f)
                for f in os.listdir(maps_path)
                if os.path.isfile(os.path.join(maps_path, f)) and f.endswith(".yaml")
            ]
            if map_files:
                latest_map_file = max(map_files, key=os.path.getmtime)
        if not latest_map_file:
            print(
                "No map file found in the maps directory. Plotting will not be available."
            )
            plot = False

    cmd_ape = [
        "evo_ape",
        "bag2",
        os.path.join(run_path, "rosbags"),
        ground_truth_topic,
        estimated_pose_topic,
        "--save_results",
        os.path.join(
            run_path, "ape_results.zip"
        ),  # Save results in the same folder as the bag file
        "--no_warnings",  # Overwrite the existing ape_results.zip file
    ]
    if align_origin:  # Align the origin of the ground truth with the estimated pose
        cmd_ape += ["--align_origin"]
    if plot:
        cmd_ape += [
            "-p",
            "--plot_mode",
            "xy",
            "--ros_map_yaml",
            latest_map_file,
        ]
    subprocess.run(cmd_ape, capture_output=True)

    cmd_rpe = [
        "evo_rpe",
        "bag2",
        os.path.join(run_path, "rosbags"),
        ground_truth_topic,
        estimated_pose_topic,
        "--save_results",
        os.path.join(
            run_path, "rpe_results.zip"
        ),  # Save results in the same folder as the bag file
        "--no_warnings",  # Overwrite the existing rpe_results.zip file
    ]
    if align_origin:  # Align the origin of the ground truth with the estimated pose
        cmd_rpe += ["--align_origin"]
    if plot:
        cmd_rpe += [
            "-p",
            "--plot_mode",
            "xy",
            "--ros_map_yaml",
            latest_map_file,
        ]
    subprocess.run(cmd_rpe, capture_output=True)

    # Load the results from ape_results.zip
    try:
        with zipfile.ZipFile(os.path.join(run_path, "ape_results.zip"), "r") as zip_ref:
            with zip_ref.open("stats.json") as stats_file:
                stats = json.load(stats_file)

        # Return the stats
        yield stats
    except Exception as e:
        print(f"Error loading APE results: {e}")
        yield None

    # and from rpe_results.zip
    try:
        with zipfile.ZipFile(os.path.join(run_path, "rpe_results.zip"), "r") as zip_ref:
            with zip_ref.open("stats.json") as stats_file:
                stats = json.load(stats_file)

        # Return the stats
        yield stats
    except Exception as e:
        print(f"Error loading RPE results: {e}")
        yield None


def process_messages(messages):
    """Read the messages and select the appropriate action for each message"""

    # List of goal, the time at which they were sent, and the time at which they were reached
    goals = []

    ground_truth_path = []  # list containing (clock, x, y)
    odometry_path = []  # list containing (clock, x, y)
    estimated_path = []  # list containing (clock, x, y)
    distances_traveled = []  # list containing (clock, eucl., man_x, man_y, rot)
    closest_obstacles = (
        []
    )  # list containing (clock, distance, angle, obstacle_detected)
    current_time = 0.0  # Initialize current time
    start_time = None
    end_time = None

    estimated_path = []

    for topic, data, timestamp in messages:
        if topic == "/clock":
            # Adjust current_time
            current_time = data.clock.sec + data.clock.nanosec * 1e-9
            if verbose:
                print(f"Current time updated: {current_time}s")

        elif topic == "/goal_sent":
            # Add the goal and the timestamp to the message.
            goals.append([(data.x, data.y), current_time, None])
            if not start_time:
                start_time = current_time
            if verbose:
                print(f"Goal sent at {current_time}s: {data.x}, {data.y}")

        elif topic == "/goal_reached":
            # Find the goal that was reached and update its time
            # It should be last goal of the list
            # With explore-lite, goals are often cancelled before being reached;
            # so the information of a goal being reached is not really useful.
            goal_point = (data.x, data.y)
            goal_found = False
            for i in range(len(goals) - 1, -1, -1):
                if goals[i][0] == goal_point:
                    # Set the current time as the reached time
                    goals[i][2] = current_time
                    goal_found = True
                    if verbose:
                        print(
                            f"Goal reached at {current_time}s, after {current_time-goals[i][1]}s: {goals[i][0]}"
                        )
                    break
            if not goal_found and verbose:
                print(
                    f"Warning: Goal reached signal received at {current_time} but no matching goal found"
                )
            # We update the end_time of the exploration
            end_time = current_time

        elif topic == "/ground_truth":
            # We save the ground truth position of the robot

            ground_truth_path.append(
                (
                    data.header.stamp.sec + data.header.stamp.nanosec * 1e-9,
                    (data.pose.pose.position.x, data.pose.pose.position.y),
                    (data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                )
            )
            if verbose:
                print(
                    f"Ground truth position at {data.header.stamp.sec + data.header.stamp.nanosec * 1e-9}s: "
                    f"{data.pose.pose.position.x}, {data.pose.pose.position.y}"
                )
        elif topic == "/robot_distances":
            # We keep track of the distance travled by the robot
            distances_traveled.append(
                (
                    current_time,
                    data.euclidean_distance,
                    data.manhattan_distance_x,
                    data.manhattan_distance_y,
                    data.rotational_distance,
                )
            )

        elif topic == "/closest_obstacle":
            # We save the closest obstacle information
            closest_obstacles.append(
                (
                    current_time,
                    data.distance,
                    data.angle,
                    data.obstacle_detected,
                )
            )

        elif topic == "/odom":
            # We save the pure odometry
            odometry_path.append(
                (
                    data.header.stamp.sec + data.header.stamp.nanosec * 1e-9,
                    (data.pose.pose.position.x, data.pose.pose.position.y),
                    (data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                )
            )

        elif topic == "/estimated_pose":
            # We save the estimated pose from the tf
            estimated_path.append(
                (
                    data.header.stamp.sec + data.header.stamp.nanosec * 1e-9,
                    (data.pose.position.x, data.pose.position.y),
                    (data.pose.orientation.z, data.pose.orientation.w),
                )
            )
            if verbose:
                print(
                    f"Estimated pose at {current_time}s: {data.pose.position.x}, {data.pose.position.y}"
                )

        # Additional method to check for exploration completion
        elif topic == "/rosout":
            if "All frontiers traversed/tried out/blacklisted, stopping." in data.msg:
                print(f"Exploration stopped at {current_time}s")
                end_time = current_time

    return (
        ground_truth_path,
        odometry_path,
        estimated_path,
        distances_traveled,
        closest_obstacles,
        goals,
        start_time,
        end_time,
    )


def main():

    global verbose

    args = parse_args()
    run_path = args.run_path
    verbose = args.verbose

    bag_path = os.path.join(run_path, "rosbags/")

    messages = read_rosbag2(bag_path)

    results = {}  # Dictionary to store results
    benchmark_path = os.path.join(run_path, "benchmark")
    if not os.path.exists(benchmark_path):
        os.makedirs(benchmark_path)

    (
        ground_truth_path,
        odometry_path,
        estimated_path,
        distances_traveled,
        closest_obstacles,
        goals,
        start_time,
        end_time,
    ) = process_messages(messages)

    # Save paths and distances to files

    ground_truth_path_dicts = [
        {
            "time": t,
            "position": {"x": pos[0], "y": pos[1]},
            "orientation": {"z": orient[0], "w": orient[1]},
        }
        for t, pos, orient in ground_truth_path
    ]

    odometry_path_dicts = [
        {
            "time": t,
            "position": {"x": pos[0], "y": pos[1]},
            "orientation": {"z": orient[0], "w": orient[1]},
        }
        for t, pos, orient in odometry_path
    ]

    estimated_path_dicts = [
        {
            "time": t,
            "position": {"x": pos[0], "y": pos[1]},
            "orientation": {"z": orient[0], "w": orient[1]},
        }
        for t, pos, orient in estimated_path
    ]

    distances_traveled_dicts = [
        {
            "time": t,
            "euclidean": eucl,
            "manhattan_x": man_x,
            "manhattan_y": man_y,
            "rotational": rot,
        }
        for t, eucl, man_x, man_y, rot in distances_traveled
    ]

    closest_obstacles_dicts = [
        {
            "time": t,
            "distance": dist,
            "angle": angle,
            "obstacle_detected": detected,
        }
        for t, dist, angle, detected in closest_obstacles
    ]

    ground_truth_path_file = os.path.join(benchmark_path, "ground_truth_path.yaml")
    odometry_path_file = os.path.join(benchmark_path, "odometry_path.yaml")
    estimated_path_file = os.path.join(benchmark_path, "estimated_path.yaml")
    distances_traveled_file = os.path.join(benchmark_path, "distances_traveled.yaml")
    closest_obstacles_file = os.path.join(benchmark_path, "closest_obstacles.yaml")

    with open(ground_truth_path_file, "w") as f:
        yaml.dump(
            ground_truth_path_dicts,
            f,
            default_flow_style=False,
            indent=4,
            sort_keys=False,
        )

    with open(odometry_path_file, "w") as f:
        yaml.dump(
            odometry_path_dicts,
            f,
            default_flow_style=False,
            indent=4,
            sort_keys=False,
        )

    with open(estimated_path_file, "w") as f:
        yaml.dump(
            estimated_path_dicts,
            f,
            default_flow_style=False,
            indent=4,
            sort_keys=False,
        )

    with open(distances_traveled_file, "w") as f:
        yaml.dump(
            distances_traveled_dicts,
            f,
            default_flow_style=False,
            indent=4,
            sort_keys=False,
        )

    with open(closest_obstacles_file, "w") as f:
        yaml.dump(
            closest_obstacles_dicts,
            f,
            default_flow_style=False,
            indent=4,
            sort_keys=False,
        )

    # Durations
    results["start_time"] = start_time
    results["end_time"] = end_time
    print(f"Exploration started at {start_time}s and ended at {end_time}s")
    if start_time and end_time:
        results["duration"] = end_time - start_time
        print(f"total duration: {end_time - start_time}s")

    # Final Distances
    results["final_distances_traveled"] = {  # We save the last distance saved
        "euclidean": distances_traveled[-1][1],
        "manhattan_x": distances_traveled[-1][2],
        "manhattan_y": distances_traveled[-1][3],
        "rotational": distances_traveled[-1][4],
    }
    print(f"Final distances traveled: {results['final_distances_traveled']}")

    # Evo metrics
    ape_stats, rpe_stats = evo_metrics(
        run_path, "/ground_truth", "/estimated_pose", plot=True, align_origin=False
    )
    if ape_stats:
        print("APE stats:")
        print(ape_stats)
        results["ape_stats"] = ape_stats
    else:
        print("No APE stats available or an error occurred.")

    if rpe_stats:
        print("RPE stats:")
        print(rpe_stats)
        results["rpe_stats"] = rpe_stats
    else:
        print("No RPE stats available or an error occurred.")

    # Progression over time
    # We want to look at coverage over time

    # Save results to YAML
    yaml_path = os.path.join(benchmark_path, "benchmark.yaml")
    with open(yaml_path, "w") as f:
        yaml.dump(results, f, default_flow_style=False, indent=4, sort_keys=False)
    print(f"Benchmark results saved to {yaml_path}")


if __name__ == "__main__":
    main()
