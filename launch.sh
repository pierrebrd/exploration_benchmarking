#!/bin/bash

# TODO Currently I dont source it !!


source /opt/ros/$ROS_DISTRO/setup.bash
source /root/exploration_benchmarking/ros2_simulation/install/setup.bash

python3 /root/exploration_benchmarking/ros2_simulation/singlerun.py "$1" "$2"
