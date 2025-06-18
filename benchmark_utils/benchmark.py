from pyexpat.errors import messages
import sys
import rclpy
from rclpy.serialization import deserialize_message
import rosbag2_py
from rosidl_runtime_py.utilities import get_message
import yaml
import os
import tf2_msgs.msg

## TODO : the use of global variables is not ideal

## Global variables

VERBOSE = False

current_time = 0.0
messages = []
# List of goal, the time at which they were sent, and the time at which they were reached
goals = []


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


def process_messages(messages):
    """Read the messages and select the appropriate action for each message"""

    global current_time
    global goals

    for topic, data, timestamp in messages:
        if topic == "/clock":
            # Adjust current_time
            current_time = data.clock.sec + data.clock.nanosec * 1e-9
            if VERBOSE:
                print(f"Current time updated: {current_time}")
        if topic == "/goal_sent":
            # Add the goal and the timestamp to the message.
            goals.append([(data.x, data.y), current_time, None])
            if VERBOSE:
                print(f"Goal sent at {current_time}: {data.x}, {data.y}")


def main():
    global messages
    global VERBOSE

    if len(sys.argv) < 2:
        print("Usage: python time_benchmark.py <path_to_rosbag> [--verbose]")
        sys.exit(1)

    if len(sys.argv) > 2:
        VERBOSE = sys.argv[2].lower() == "--verbose"

    file_path = sys.argv[1]

    messages = read_rosbag2(file_path)

    process_messages(messages)

    # for topic, data, timestamp in messages:
    #     print(f"Topic: {topic}, Timestamp: {timestamp}")


if __name__ == "__main__":
    main()
