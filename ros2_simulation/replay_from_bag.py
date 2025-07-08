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

# Import functions from other py scripts
from singlerun import (
    kill_process,
    launch_rosbag2_recording,
    launch_rosbag2_playing,
    launch_roslaunchfile,
    launch_rosnode,
    launch_generic,
    save_map,
    save_maps_thread,
)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Replay a ROS2 simulation run using different parameters and algorithms, based on a YAML config file."
    )
    parser.add_argument(
        "config_path",
        help="Path to the YAML config file containing the parameters for the simulation run.",
    )
    parser.add_argument(
        "input_run_folder",
        help="Path to the folder containing the input run data. (This folder should contain a 'rosbags' subfolder)",
    )
    return parser.parse_args()


def main():

    args = parse_args()
    config_path = os.path.abspath(args.config_path)
    input_run_folder = os.path.abspath(args.input_run_folder)

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
            "rosbag2_recorded_topics",
            "rosbag2_played_topics",
        ]
    ), "Missing required parameters"

    # Extract parameters
    rosbag2_recorded_topics = config_data["rosbag2_recorded_topics"]
    rosbag2_played_topics = config_data["rosbag2_played_topics"]
    map_saver_interval = config_data.get("map_saver_interval", 0)
    rate = config_data.get("rate", None)
    slam = config_data.get("slam", None)
    exploration = config_data.get("exploration", None)
    navigation = config_data.get("navigation", None)
    simulation = config_data.get("simulation", None)
    additional_processes = config_data.get("additional_processes", [])

    try:
        # Make folders
        output_folder = os.path.join(input_run_folder, "..")
        if not os.path.exists(output_folder):
            os.makedirs(output_folder)

        # Create the folder for this run
        run_name = os.path.basename(input_run_folder) + "_replay"
        run_folder = os.path.join(output_folder, run_name)
        i = 1
        while os.path.exists(run_folder):
            run_name = os.path.basename(input_run_folder) + "_replay" + f"_{i}"
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
                    params_dest_file = os.path.join(
                        params_folder, os.path.basename(dict["params_file"])
                    )
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

        # We first launch the tf filter node
        tf_filter_process = launch_rosnode("exploration_benchmarking", "tf_filter")
        running_processes.append(tf_filter_process)

        # Then the rosbag2 recording node
        rosbag2_recording_process = launch_rosbag2_recording(
            rosbag2_recorded_topics, rosbags_folder
        )
        running_processes.append(rosbag2_recording_process)

        # Launch the additional processes if specified
        for additional_process in additional_processes:
            running_processes.append(launch_generic(additional_process))
            # time.sleep(2) # Add delay?

        # Then we launch the rosbag2 playing node
        rosbag2_playing_process = launch_rosbag2_playing(
            rosbag2_played_topics,
            os.path.join(input_run_folder, "rosbags"),
            remappings={"/tf": "tf_unfiltered"},
            rate=rate,
        )
        running_processes.append(rosbag2_playing_process)

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

        # Wait for the rosbag playing process to finish
        rosbag2_playing_process.wait()
        print("Rosbag2 playing process has ended.")
        # Wait for 5 seconds before proceeding
        time.sleep(5)

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
        final_map_process = save_map(maps_folder, f"map_final_{run_name}")
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
