from html import parser
from pyexpat.errors import messages
import sys
import time

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
import argparse
from rcl_interfaces.msg import Log


def parse_args():
    parser = argparse.ArgumentParser(description="Benchmark a ROS2 simulation run")
    parser.add_argument(
        "run_path", help="Path to the run folder (should contain a 'rosbags' subfolder)"
    )
    return parser.parse_args()


def read_rosbag2(file_path):
    """return a list messages on the /rosout topic in the rosbag2 file"""
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
        if topic == "/rosout":
            msg_type = get_message(typename(topic))
            msg = deserialize_message(data, msg_type)
            yield topic, msg, timestamp
    del reader


def process_messages(messages):
    # Map ROS2 log levels to their string representations
    level_map = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}

    for topic, data, timestamp in messages:
        assert isinstance(data, Log), "Expected Log message type"
        level_name = level_map.get(data.level, f"UNKNOWN({data.level})")
        print(
            f"[{level_name}] [{data.stamp.sec}.{data.stamp.nanosec}] [{data.name}] {data.msg}"
        )


def main():

    args = parse_args()
    run_path = os.path.abspath(args.run_path)

    bag_path = os.path.join(run_path, "rosbags/")

    messages = read_rosbag2(bag_path)

    process_messages(messages)


if __name__ == "__main__":
    main()
