import subprocess
import os
from concurrent.futures import ThreadPoolExecutor
import argparse


def spawn_container(config_file: str, i: int, mount_folder):
    domain_id = 100 + i
    launch_cmd = [
        "docker",
        "run",
        "-it",
        "-e",
        f"ROS_DOMAIN_ID={domain_id}",
        "-v",
        f"{mount_folder}:/root/exploration_benchmarking/",
        "--entrypoint",
        "/bin/bash -i /root/exploration_benchmarking/launch.sh",
        "ros2humble:exploration_benchmarking",
        config_file,
        "",
    ]
    subprocess.run(launch_cmd)


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch simulations for all the configs in the given folder. The simulations are launched in docker containers. Path are relative to the terminal working directory, not the py script location."
    )
    parser.add_argument(
        "configs_folder",
        help="Path to the folder containing the YAML config files for the simulation runs.",
    )
    parser.add_argument(
        "output_folder",
        nargs="?",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "runs"),
        help="Optional output folder, that will contain the runs folder",
    )
    parser.add_argument(
        "--n_containers", type=int, default=5, help="Number of containers to launch"
    )
    return parser.parse_args()


def main():
    args = parse_args()
    configs_folder = os.path.abspath(args.configs_folder)
    output_folder = os.path.abspath(args.output_folder)
    n_containers = args.n_containers

    mount_folder = os.path.abspath(os.path.join(os.getcwd(), "../"))

    config_files = os.listdir(configs_folder)
    config_files.sort()

    config_files = [file for file in config_files if file.endswith(".yaml")]

    pool = ThreadPoolExecutor(max_workers=n_containers)
    try:
        futures = []
        for i, config_file in enumerate(config_files):
            futures.append(pool.submit(spawn_container, config_file, i, mount_folder))
        pool.shutdown(wait=True)
    except Exception as e:
        print(f"Error: {e}")
        pool.shutdown(wait=False)
        subprocess.run("docker kill $(docker ps -q)", shell=True)


if __name__ == "__main__":
    main()
