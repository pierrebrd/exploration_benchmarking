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
    cmd = ["ros2", "bag", "record", "--log-level", "info", "--output", folder] + topics
    print("Starting rosbag2 recording...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def launch_roslaunchfile(package, launch_file, args):
    cmd = ["ros2", "launch", package, launch_file]
    print(f"Launching launchfile {package}/{launch_file}...")
    p = subprocess.Popen(cmd, preexec_fn=os.setsid)
    return p


def save_maps(interval, folder, stop_event):

    # TODO: doesnt respect use_sim_time,
    print("Starting map saving thread...")
    time.sleep(20)  # Initial delay to ensure the /map topic is available
    index = 0

    while not stop_event.is_set():
        print("coucou")

        map_path = os.path.join(folder, f"map_{int(index)}")
        save_cmd = [
            "ros2",
            "run",
            "nav2_map_server",
            "map_saver_cli",
            "-f",
            map_path,
            "--fmt",
            "pgm",
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
    print("Map saving thread stopping.")


def main():

    try:

        # Make folders
        current_directory = os.path.dirname(os.path.abspath(__file__))
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

        # Init ROS2
        rclpy.init()

        # Init stop event for threads
        stop_event = threading.Event()

        # We first launch the rosbag2 recording node
        rosbag2_topics_list = [
            "/odom",  # robot odometry, probably not needed
            "/ground_truth",  # ground truth robot position
            "/rosout",  # logs
            "/goal_sent",  # goals sent by the exploration algorithmFalse
            "/goal_reached",  # goals reached by the exploration algorithm
            "/clock",  # clock, to use sim time
            # "/base_scan",
            # "/tf",
            # "/tf_static",
            # "-a",
        ]
        rosbag2_process = launch_rosbag2(rosbag2_topics_list, rosbags_folder)

        # Then we launch the map saver server
        # Currently we dont launch a static map saver server, a map saver server is launched and destroyed everytime we call map_saver_cli
        map_saver_interval = 10  # seconds
        map_saver_thread = threading.Thread(
            target=save_maps,
            args=(map_saver_interval, maps_folder, stop_event),
            daemon=True,
        )
        map_saver_thread.start()

        # Launch the simulation + exploration with the launchfile
        simulation_args = ""  # string containing the arguments for the launch file, in the form "arg1:=value1 arg2:=value2"
        simulation_exploration_process = launch_roslaunchfile(
            "exploration_benchmarking",
            "simulation_exploration.launch.py",
            simulation_args,
        )

        # TODO: completion node, rviz,...
        #

        # Wait for the simulation to complete
        simulation_exploration_process.wait()
        print("Simulation process has ended.")

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

        # Kill the simulation process if it's still running
        try:
            if simulation_exploration_process.poll() is None:
                kill_process(simulation_exploration_process)
                print("Simulation process terminated.")
        except Exception as e:
            print(f"Failed to terminate simulation process: {e}")

        # Kill the rosbag recording
        try:
            if rosbag2_process.poll() is None:
                kill_process(rosbag2_process)
                print("Rosbag recording terminated.")
        except Exception as e:
            print(f"Failed to terminate rosbag process: {e}")

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
    # if len(sys.argv) < 2:
    #     print("Usage: python3 singlerun.py path_to_world.world")
    #     sys.exit(1)

    # world_path = os.path.abspath(sys.argv[1])
    # project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
    # explore_worlds(project_root, world_path)
    main()
