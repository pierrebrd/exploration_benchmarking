# This file is used to generate the configs that can be find in the configs folder.

import sys
import os
import yaml
import datetime
import time

# def extract_uppercase(data):
#     uppercase_configs = {}


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


def main(input_yaml_path):
    current_directory = os.path.dirname(os.path.abspath(__file__))
    configs_folder = os.path.join(current_directory, "configs")

    # Read the input YAML file
    try:
        with open(input_yaml_path, "r") as f:
            input_data = yaml.safe_load(f)
    except Exception as e:
        print(f"Failed to read input YAML file {input_yaml_path}: {e}")
        sys.exit(1)

    # TODO : assert things?

    # Create the folder for the generated configs
    generated_name = datetime.datetime.now().strftime("configs_%Y_%m_%d_%H_%M")
    generated_folder = os.path.join(configs_folder, generated_name)
    # Check if the run folder already exists, if so, append _1, _2, etc.
    i = 1
    while os.path.exists(generated_folder):
        generated_name = (
            datetime.datetime.now().strftime("configs_%Y_%m_%d_%H_%M") + f"_{i}"
        )
        generated_folder = os.path.join(configs_folder, generated_name)
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


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python generate_configs.py <path_to_input_yaml>")
        sys.exit(1)
    input_yaml_path = sys.argv[1]
    if not os.path.exists(input_yaml_path):
        print(f"Input YAML file does not exist: {input_yaml_path}")
        sys.exit(1)
    main(input_yaml_path)
