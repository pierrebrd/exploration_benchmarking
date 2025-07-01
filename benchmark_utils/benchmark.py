from pyexpat.errors import messages
import sys

import evo.main_ape
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml
import os
import tf2_msgs.msg
import evo
import subprocess
import json
import zipfile

## Global variables

VERBOSE = False


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
    TODO : currently there is a problem because the ground_truth starts at (0, 0) instead of the initial poisiton of the robot
    defined in the world file. This results in the ground_truth not "fitting" into the map image. --align_origin is useful to
    aligh the odom with the ground truth, but then nothing fits into the map image.
    Possible solution: edit the world file OR use a ground_truth that starts at the initial position OR set a static TF?
    """
    # We use evo to compute APE (and RPE)
    # Easiest way I found is to call the bash command directly

    # Find the most recent file in the 'maps' directory
    maps_path = os.path.join(os.path.dirname(run_path), "maps")

    if plot:
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
        os.path.join(os.path.dirname(run_path), "rosbags"),
        ground_truth_topic,
        estimated_pose_topic,
        "--save_results",
        os.path.join(
            os.path.dirname(run_path), "ape_results.zip"
        ),  # Save results in the same folder as the bag file
        "--no_warnings",  # Overwrite the existing ape_results.zip file
    ]
    if align_origin:
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
        os.path.join(os.path.dirname(run_path), "rosbags"),
        ground_truth_topic,
        estimated_pose_topic,
        "--save_results",
        os.path.join(
            os.path.dirname(run_path), "rpe_results.zip"
        ),  # Save results in the same folder as the bag file
        "--no_warnings",  # Overwrite the existing rpe_results.zip file
    ]
    if align_origin:
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

    # TODO : set other parameters (--pose_relation? --delta, --delta_tol)

    # Load the results from ape_results.zip
    try:
        with zipfile.ZipFile(
            os.path.join(os.path.dirname(run_path), "ape_results.zip"), "r"
        ) as zip_ref:
            with zip_ref.open("stats.json") as stats_file:
                stats = json.load(stats_file)

        # Return the stats
        yield stats
    except Exception as e:
        print(f"Error loading APE results: {e}")
        yield None

    # and from rpe_results.zip
    try:
        with zipfile.ZipFile(
            os.path.join(os.path.dirname(run_path), "rpe_results.zip"), "r"
        ) as zip_ref:
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

    robot_path = []
    current_time = 0.0  # Initialize current time
    start_time = None
    end_time = None
    # previous_pose = None  # used to determine when the robot actually starts moving

    for topic, data, timestamp in messages:
        if topic == "/clock":
            # Adjust current_time
            current_time = data.clock.sec + data.clock.nanosec * 1e-9
            if VERBOSE:
                print(f"Current time updated: {current_time}s")

        elif topic == "/goal_sent":
            # Add the goal and the timestamp to the message.
            goals.append([(data.x, data.y), current_time, None])
            if not start_time:
                start_time = current_time
            if VERBOSE:
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
                    if VERBOSE:
                        print(
                            f"Goal reached at {current_time}s, after {current_time-goals[i][1]}s: {goals[i][0]}"
                        )
                    break
            if not goal_found and VERBOSE:
                print(
                    f"Warning: Goal reached signal received at {current_time} but no matching goal found"
                )
            # We update the end_time of the exploration
            end_time = current_time

        elif topic == "/ground_truth":
            # We save the ground truth position of the robot

            robot_path.append(
                (
                    data.header.stamp.sec + data.header.stamp.nanosec * 1e-9,
                    (data.pose.pose.position.x, data.pose.pose.position.y),
                    (data.pose.pose.orientation.z, data.pose.pose.orientation.w),
                )
            )
            if VERBOSE:
                print(
                    f"Ground truth position at {data.header.stamp.sec + data.header.stamp.nanosec * 1e-9}s: "
                    f"{data.pose.pose.position.x}, {data.pose.pose.position.y}"
                )

    return (robot_path, goals, start_time, end_time)


def main():
    global VERBOSE

    if len(sys.argv) < 2:
        print("Usage: python time_benchmark.py <path_to_rosbag> [--verbose]")
        sys.exit(1)

    if len(sys.argv) > 2:
        VERBOSE = sys.argv[2].lower() == "--verbose"

    run_path = sys.argv[1]
    bag_path = os.path.join(run_path, "rosbags/")

    messages = read_rosbag2(bag_path)

    robot_path, goals, start_time, end_time = process_messages(messages)

    # print(goals)
    print(start_time, end_time)
    if start_time and end_time:
        print(
            f"Exploration started at {start_time}s and ended at {end_time}s, "
            f"total duration: {end_time - start_time}s"
        )

    ape_stats, rpe_stats = evo_metrics(
        run_path, "/ground_truth", "/tf:map.base_link", plot=True, align_origin=False
    )
    if ape_stats:
        print("APE stats:")
        print(ape_stats)
    else:
        print("No APE stats available or an error occurred.")

    if rpe_stats:
        print("RPE stats:")
        print(rpe_stats)
    else:
        print("No RPE stats available or an error occurred.")


if __name__ == "__main__":
    main()

# TODO: save all the results in a file (json or yaml)
