import cmd
import datetime
import os
import sys
import signal
import time
import subprocess
import threading
from PIL import Image

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

from visualization_msgs.msg import MarkerArray
from rclpy.executors import MultiThreadedExecutor
import yaml
import shutil
import argparse


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch a ROS2 simulation run based on a YAML config file."
    )
    parser.add_argument(
        "config_path",
        help="Path to the YAML config file containing the parameters for the simulation run.",
    )
    parser.add_argument(
        "output_folder",
        nargs="?",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs"),
        help="Optional output folder, that will contain the run folder",
    )
    return parser.parse_args()


def kill_process(p):
    try:
        pid = os.getpgid(p.pid)
        os.killpg(pid, signal.SIGINT)
        for _ in range(100):
            if p.poll() is not None:
                return
            time.sleep(0.1)
        os.killpg(pid, signal.SIGTERM)
        for _ in range(50):
            if p.poll() is not None:
                print("Process exited after SIGTERM!!")
                return
            time.sleep(0.1)
        os.killpg(pid, signal.SIGKILL)
    except Exception as e:
        print(f"Failed to kill process: {e}")
    print("Process needed to be killed with SIGKILL!!")


def explore_worlds(project_path, world_file):
    out_dir = os.path.join(project_path, "runs", "outputs")
    world_name = os.path.basename(world_file).replace(".world", "")
    folder = os.path.join(out_dir, world_name)


def launch_rosbag2_recording(topics, folder, use_sim_time=False):
    cmd = [
        "ros2",
        "bag",
        "record",
        "--log-level",
        "info",
        "--output",
        folder,
    ] + topics
    if use_sim_time:
        cmd += ["--use-sim-time"]
    print("Starting rosbag2 recording...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_rosbag2_playing(topics, folder, remappings: dict = None, rate=None):
    cmd = [
        "ros2",
        "bag",
        "play",
        "--log-level",
        "info",
        folder,
        "--topics",
    ] + topics
    if remappings:
        cmd += ["--remap"]
        for key, value in remappings.items():
            cmd += [f"{key}:={value}"]
    if rate:
        cmd += ["--rate", str(rate)]
    print("Starting rosbag2 playing...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_roslaunchfile(
    package,
    launchfile,
    params_file=None,
    params_file_argument="params_file",
    launchfile_args=None,
    ros_args=None,
):
    cmd = ["ros2", "launch", package, launchfile]
    if launchfile_args:
        for key, value in launchfile_args.items():
            cmd += [f"{key}:={str(value)}"]
    if params_file:
        # The launchfile should 'listen' to a params_file argument
        cmd += [f"{params_file_argument}:={params_file}"]
    if ros_args:
        cmd += ["--ros-args"] + ros_args

    print(f"Launching launchfile {package}/{launchfile}...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_rosnode(
    package, node, params_file=None, other_params=None, args=None, ros_args=None
):
    cmd = ["ros2", "run", package, node]
    if args:
        cmd += args
    if params_file:
        cmd += ["--ros-args", "--params-file", params_file]
    if other_params:
        cmd += ["--ros-args"]
        for key, value in other_params.items():
            cmd += ["-p", f"{key}:={value}"]
    if ros_args:
        cmd += ["--ros-args"] + ros_args
    print(f"Launching ROS2 node {package}/{node}...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_generic(dict: dict, other_params=None):
    if dict["is_launch_file"]:
        process = launch_roslaunchfile(
            dict["package"],  # Will throw an error if the key is not found
            dict["name"],
            params_file=dict.get("params_file", None),
            params_file_argument=dict.get("params_file_argument", None),
            launchfile_args=dict.get("launchfile_args", {}),
            ros_args=dict.get("ros_args", []),
        )
    else:
        # TODO : use other_params
        process = launch_rosnode(
            dict["package"],
            dict["name"],
            params_file=dict.get("params_file", None),
            # other_params=,
            ros_args=dict.get("ros_args", []),
        )
    return process


def save_map(folder, map_name):
    """save a single map"""
    map_path = os.path.join(folder, map_name)
    save_cmd = [
        "ros2",
        "run",
        "nav2_map_server",
        "map_saver_cli",
        "-f",
        map_path,
        "--fmt",
        "png",
        "--ros-args",
        "--log-level",
        "info",
    ]
    process = subprocess.Popen(save_cmd, preexec_fn=os.setsid)
    return process


def save_maps_thread(interval, folder, stop_event):

    # TODO: doesnt respect use_sim_time,
    print("Starting map saving thread...")
    time.sleep(20)  # Initial delay to ensure the /map topic is available
    index = 0

    while not stop_event.is_set():

        process = save_map(folder, f"map_{int(index)}")

        index += 1

        # Wait, while looking for the stop event
        for _ in range(int(interval * 10)):
            if stop_event.is_set():
                break
            time.sleep(0.1)

    process.wait(timeout=10)
    print("Map saving thread stopping.")


def main():

    args = parse_args()
    config_path = os.path.abspath(args.config_path)
    output_folder = args.output_folder

    current_directory = os.path.dirname(os.path.abspath(__file__))

    # Read Parameters

    try:
        with open(config_path, "r") as f:
            config_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to read config file {config_path}: {e}")
        sys.exit(1)

    assert all(
        key in config_data
        for key in [
            "world_name",
            "rosbag2_recorded_topics",
            "slam",
            "exploration",
            "navigation",
            "simulation",
        ]
    ), "Missing required parameters"

    # Extract parameters
    world_name = config_data["world_name"]
    worlds_folder = config_data.get(
        "worlds_folder", os.path.join(current_directory, "..", "worlds")
    )
    rosbag2_recorded_topics = config_data["rosbag2_recorded_topics"]
    map_saver_interval = config_data.get("map_saver_interval", 0)
    slam = config_data.get("slam", None)
    exploration = config_data.get("exploration", None)
    navigation = config_data.get("navigation", None)
    simulation = config_data.get("simulation", None)
    additional_processes = config_data.get("additional_processes", [])

    # Check if the world file exists
    world_path = os.path.join(worlds_folder, world_name)
    simulation["launchfile_args"]["world"] = world_path
    # TODO : put the world choice in the the simulation params_file instead?

    try:
        # Make folders
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # Create the folder for this run
        run_name = datetime.datetime.now().strftime("run_%Y_%m_%d_%H_%M")
        run_folder = os.path.join(output_folder, run_name)
        # Check if the run folder already exists, if so, append _1, _2, etc.
        i = 1
        while os.path.exists(run_folder):
            run_name = datetime.datetime.now().strftime("run_%Y_%m_%d_%H_%M") + f"_{i}"
            run_folder = os.path.join(output_folder, run_name)
            i += 1
        os.makedirs(run_folder)
        del i

        # Create the necessary subfolders
        maps_folder = os.path.join(run_folder, "maps")
        if not os.path.exists(maps_folder):
            os.makedirs(maps_folder)
        rosbags_folder = os.path.join(run_folder, "rosbags")
        params_folder = os.path.join(run_folder, "params")

        # Copy the parameters files
        if not os.path.exists(params_folder):
            os.makedirs(params_folder)
        config_dest_path = os.path.join(run_folder, os.path.basename(config_path))
        try:
            shutil.copy(config_path, config_dest_path)
        except Exception as e:
            print(f"Failed to copy config file to {config_dest_path}: {e}")
        for dict in [slam, exploration, navigation, simulation]:
            if dict and dict.get("params_file", None):
                try:
                    params_dest_file = os.path.join(run_folder, dict["params_file"])
                    os.makedirs(os.path.dirname(params_dest_file), exist_ok=True)
                    shutil.copy(dict["params_file"], params_dest_file)
                except Exception as e:
                    print(
                        f"Failed to copy parameter file {dict['params_file']} to {params_dest_file}: {e}"
                    )
        # TODO : maybe dump the params using the ros2 param dump command? for each node ?

        # Init ROS2
        rclpy.init()

        # Init stop event for threads
        stop_event = threading.Event()

        # List to store running processes
        running_processes = []

        # Launch rviz if specified in the parameters
        if "rviz_config" in config_data:
            try:
                rviz_config = config_data["rviz_config"]
                rviz_process = launch_rosnode(
                    package="rviz2",
                    node="rviz2",
                    args=["-d", rviz_config],
                    other_params={"use_sim_time": "true"},
                )
                running_processes.append(rviz_process)
            except Exception as e:
                print(
                    f"Failed to launch RViz with config {config_data['rviz_config']}: {e}"
                )

        # We first launch the rosbag2 recording node
        rosbag2_process = launch_rosbag2_recording(
            rosbag2_recorded_topics, rosbags_folder
        )
        running_processes.append(rosbag2_process)

        # Then we launch the map saver server
        # Currently we dont launch a static map saver server, a map saver server is launched and destroyed everytime we call map_saver_cli
        if map_saver_interval > 0:
            map_saver_thread = threading.Thread(
                target=save_maps_thread,
                args=(map_saver_interval, maps_folder, stop_event),
                daemon=True,
            )
            map_saver_thread.start()

        time.sleep(2)

        # Launch the additional processes if specified
        for additional_process in additional_processes:
            running_processes.append(launch_generic(additional_process))
            # time.sleep(2) # Add delay?

        # Launch the simulation platform
        if simulation:
            simulation_process = launch_generic(simulation)
            running_processes.append(simulation_process)
            time.sleep(2)

        # Launch the SLAM
        if slam:
            slam_process = launch_generic(slam)
            running_processes.append(slam_process)
            time.sleep(2)

        # Launch the navigation stack
        if navigation:
            navigation_process = launch_generic(navigation)
            running_processes.append(navigation_process)
            time.sleep(2)

        # Launch the exploration algorithm
        if exploration:
            exploration_process = launch_generic(exploration)
            running_processes.append(exploration_process)
            time.sleep(2)

        # Wait for the exploration to complete
        exploration_process.wait()
        print("Exploration process has ended.")

    except KeyboardInterrupt:
        print("\nKeyboard interrupt received, shutting down processes...")
    except Exception as e:
        print(f"An error occurred: {e}")
        print("Shutting down processes due to error...")
    finally:
        # Clean up all processes
        print("Cleaning up processes...")

        # Stop the map saver thread
        try:
            stop_event.set()
            map_saver_thread.join()
            print("Map saver thread stopped.")
        except Exception as e:
            print(f"Failed to stop map saver thread: {e}")

        # Save the final map
        final_map_process = save_map(maps_folder, f"map_final_{run_name}_{world_name}")
        running_processes.append(final_map_process)
        # Ensure the final map is saved before killing the other processes
        final_map_process.wait(timeout=10)

        # Kill the running processes
        running_processes.reverse()
        for process in running_processes:
            try:
                if process.poll() is None:
                    kill_process(process)
                    print(f"{process} terminated.")
            except Exception as e:
                print(f"Failed to terminate {process}: {e}")

        # Shutdown ROS2
        try:
            if rclpy.ok():
                rclpy.shutdown()
                print("ROS2 node shutdown.")
        except Exception as e:
            print(f"Failed to shutdown ROS2: {e}")

        print("All processes terminated successfully.")

        # Give processes time to fully terminate
        time.sleep(1)


if __name__ == "__main__":
    # project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # explore_worlds(project_root, world_path)
    main()
