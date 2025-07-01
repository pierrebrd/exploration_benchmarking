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


def launch_rosbag2(topics, folder):
    cmd = [
        "ros2",
        "bag",
        "record",
        "--log-level",
        "info",
        "--output",
        folder,
        "--use-sim-time",
    ] + topics
    print("Starting rosbag2 recording...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_roslaunchfile(
    package, launchfile, params_file=None, launchfile_args=None, ros_args=None
):
    cmd = ["ros2", "launch", package, launchfile]
    if launchfile_args:
        for key, value in launchfile_args.items():
            cmd += [f"{key}:={str(value)}"]
    if params_file:
        # The launchfile should 'listen' to a params_file argument
        cmd += [f"params_file:={params_file}"]
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


def launch_generic(dict, other_params=None):
    # TODO : use other_params
    if dict["is_launch_file"]:
        process = launch_roslaunchfile(
            dict["package"],
            dict["name"],
            params_file=dict["params_file"],
            launchfile_args=dict["launchfile_args"],
            ros_args=dict["ros_args"],
        )
    else:
        process = launch_rosnode(
            dict["package"],
            dict["name"],
            params_file=dict["params_file"],
            # other_params=,
            ros_args=dict["ros_args"],
        )
    return process


def save_maps(interval, folder, stop_event):

    # TODO: doesnt respect use_sim_time,
    print("Starting map saving thread...")
    time.sleep(20)  # Initial delay to ensure the /map topic is available
    index = 0

    while not stop_event.is_set():
        map_path = os.path.join(folder, f"map_{int(index)}")
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

        index += 1

        # Wait, while looking for the stop event
        for _ in range(int(interval * 10)):
            if stop_event.is_set():
                break
            time.sleep(0.1)
    # Save final map
    map_path = os.path.join(folder, f"map_final")
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
    print("Map saving thread stopping.")


def main(param_path, project_root):

    current_directory = os.path.dirname(os.path.abspath(__file__))

    # Read Parameters

    try:
        with open(param_path, "r") as f:
            params = yaml.safe_load(f)
            # Example: extract variables from the loaded params
    except Exception as e:
        print(f"Failed to read parameters file {param_path}: {e}")
        sys.exit(1)

    assert all(
        key in params
        for key in [
            "world_name",
            "rosbag2_topics_list",
            "map_saver_interval",
            "slam",
            "exploration",
            "navigation",
            "simulation",
        ]
    ), "Missing required parameters"

    # Extract parameters
    world_name = params["world_name"]
    rosbag2_topics_list = params["rosbag2_topics_list"]
    map_saver_interval = params["map_saver_interval"]
    slam = params["slam"]
    exploration = params["exploration"]
    navigation = params["navigation"]
    simulation = params["simulation"]

    # Check if the world file exists
    world_path = os.path.join(current_directory, "..", "worlds", world_name)
    simulation["launchfile_args"]["world"] = world_path
    # TODO : put the world choice in the the simulation params_file instead?

    try:
        # Make folders
        runs_folder = os.path.join(current_directory, "..", "runs")
        if not os.path.exists(runs_folder):
            os.makedirs(runs_folder)

        # Create the folder for this run
        run_name = datetime.datetime.now().strftime("run_%Y_%m_%d_%H_%M")
        run_folder = os.path.join(runs_folder, run_name)
        while os.path.exists(run_folder):
            run_name += "_1"
            run_folder = os.path.join(runs_folder, run_name)
        os.makedirs(run_folder)
        # Create the necessary subfolders
        maps_folder = os.path.join(run_folder, "maps")
        if not os.path.exists(maps_folder):
            os.makedirs(maps_folder)
        rosbags_folder = os.path.join(run_folder, "rosbags")
        params_folder = os.path.join(run_folder, "params")

        # Copy the parameters files
        if not os.path.exists(params_folder):
            os.makedirs(params_folder)
        params_dest_path = os.path.join(params_folder, os.path.basename(param_path))
        try:
            shutil.copy(param_path, params_dest_path)
        except Exception as e:
            print(f"Failed to copy parameter file to {params_dest_path}: {e}")
        for dict in [slam, exploration, navigation, simulation]:
            if dict["params_file"]:
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
        if "rviz_config" in params:
            try:
                rviz_config = params["rviz_config"]
                # rviz_process = launch_roslaunchfile(
                #     package="exploration_benchmarking",
                #     launchfile="basic_rviz.launch.py",
                #     launchfile_args={
                #         "config": rviz_config,
                #         "use_sim_time": "true",
                #     },
                # )
                rviz_process = launch_rosnode(
                    package="rviz2",
                    node="rviz2",
                    args=["-d", rviz_config],
                    other_params={"use_sim_time": "true"},
                )
                running_processes.append(rviz_process)
            except Exception as e:
                print(f"Failed to launch RViz with config {params['rviz_config']}: {e}")

        # We first launch the rosbag2 recording node
        rosbag2_process = launch_rosbag2(rosbag2_topics_list, rosbags_folder)
        running_processes.append(rosbag2_process)

        # Then we launch the map saver server
        # Currently we dont launch a static map saver server, a map saver server is launched and destroyed everytime we call map_saver_cli
        map_saver_thread = threading.Thread(
            target=save_maps,
            args=(map_saver_interval, maps_folder, stop_event),
            daemon=True,
        )
        map_saver_thread.start()

        time.sleep(2)

        # Launch the simulation platform
        simulation_process = launch_generic(simulation)
        running_processes.append(simulation_process)
        time.sleep(2)

        # Launch the SLAM
        slam_process = launch_generic(slam)
        running_processes.append(slam_process)
        time.sleep(2)

        # Launch the navigation stack
        navigation_process = launch_generic(navigation)
        running_processes.append(navigation_process)
        time.sleep(2)

        # Launch the exploration algorithm
        exploration_process = launch_generic(exploration)
        running_processes.append(exploration_process)
        time.sleep(2)

        # TODO: completion node, rviz,...
        #

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
    if len(sys.argv) < 2:
        print("Usage: python3 singlerun.py relative_path_to_param.yaml")
        sys.exit(1)

    param_path = os.path.abspath(sys.argv[1])
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # explore_worlds(project_root, world_path)
    main(param_path, project_root)
