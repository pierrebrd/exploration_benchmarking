import sys
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml
import os
import tf2_msgs.msg


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


def main():
    if len(sys.argv) < 2:
        print("Usage: python time_benchmark.py <path_to_rosbag>")
        sys.exit(1)

    file_path = sys.argv[1]

    messages = read_rosbag2(file_path)
    # for topic, data, timestamp in messages:
    #     print(data)
    for topic, data, timestamp in messages:
        if topic == "/clock":
            print(f"Timestamp: {timestamp}, Topic: {topic}, Message: {data}")


if __name__ == "__main__":
    main()
