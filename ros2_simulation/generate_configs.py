# This file is used to generate the configs that can be find in the configs folder.

import sys
import os
import yaml
import datetime
import time
import argparse

# def extract_uppercase(data):
#     uppercase_configs = {}


def parse_args():
    parser = argparse.ArgumentParser(
        description="Generate config files from a YAML template.Path are relative to the terminal working directory, not the py script location."
    )
    parser.add_argument("input_yaml_path", help="Path to the input YAML file")
    parser.add_argument(
        "output_folder",
        nargs="?",
        default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "configs"),
        help="Optional output folder for generated configs",
    )
    return parser.parse_args()


def recursive_config(data):
    if isinstance(data, dict):
        configs = [{}]
        for key, value in data.items():
            if key.isupper():  # We have multiple configurations in a list
                child_configs = []
                for child in value:  # For every possibility
                    new_data = {}
                    new_data[key.lower()] = child
                    child_configs += recursive_config(
                        new_data
                    )  # We add the child configs
                tmp_configs = []
                for config in configs:
                    for child_config in child_configs:
                        new_config = config.copy()
                        new_config.update(child_config)
                        tmp_configs.append(new_config)
                configs = tmp_configs
            else:  # normal config
                if isinstance(value, dict):
                    child_configs = recursive_config(value)
                    tmp_configs = []
                    for config in configs:
                        for child_config in child_configs:
                            new_config = config.copy()
                            new_config[key] = child_config
                            tmp_configs.append(new_config)
                    configs = tmp_configs
                else:  # simply add (key,value) to the configs
                    for config in configs:
                        config[key] = value
        return configs
    else:
        print(f"Error, not a dict : {data} ")


def main():

    args = parse_args()
    input_yaml_path = os.path.abspath(args.input_yaml_path)
    output_folder = os.path.abspath(args.output_folder)

    if not os.path.exists(input_yaml_path):
        print(f"Input YAML file does not exist: {input_yaml_path}")
        sys.exit(1)

    if not os.path.exists(output_folder):
        try:
            os.makedirs(output_folder)
        except Exception as e:
            print(f"Failed to create output folder {output_folder}: {e}")
            sys.exit(1)

    # Read the input YAML file
    try:
        with open(input_yaml_path, "r") as f:
            input_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to read input YAML file {input_yaml_path}: {e}")
        sys.exit(1)

    # Create the folder for the generated configs
    generated_name = datetime.datetime.now().strftime("configs_%Y_%m_%d_%H_%M")
    generated_folder = os.path.join(output_folder, generated_name)
    # Check if the run folder already exists, if so, append _1, _2, etc.
    i = 1
    while os.path.exists(generated_folder):
        generated_name = (
            datetime.datetime.now().strftime("configs_%Y_%m_%d_%H_%M") + f"_{i}"
        )
        generated_folder = os.path.join(output_folder, generated_name)
        i += 1
    os.makedirs(generated_folder)
    del i

    # Create the configs
    # When we detect a key in uppercase, this means it is a list of configs: we create configs for each items of the list
    configs = recursive_config(input_data)
    for i, config in enumerate(configs):
        output_path = os.path.join(generated_folder, f"config_{i}.yaml")
        with open(output_path, "w") as f:
            yaml.dump(config, f, default_flow_style=False, indent=4, sort_keys=False)
    print(f"Generated {len(configs)} configs in {generated_folder}")


if __name__ == "__main__":
    main()
