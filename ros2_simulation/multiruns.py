import os

import argparse

from singlerun import singlerun


def parse_args():
    parser = argparse.ArgumentParser(
        description="Launch simulations for all the configs in the given folder. The simulations are launched one after the other for now. Path are relative to the terminal working directory, not the py script location."
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
    return parser.parse_args()


def main():
    args = parse_args()
    configs_folder = os.path.abspath(args.configs_folder)
    output_folder = os.path.abspath(args.output_folder)

    for config_file in os.listdir(configs_folder):
        if config_file.endswith(".yaml"):
            config_path = os.path.join(configs_folder, config_file)
            singlerun(config_path, output_folder)


if __name__ == "__main__":
    main()
