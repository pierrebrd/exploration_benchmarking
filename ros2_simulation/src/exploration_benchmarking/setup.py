from setuptools import find_packages, setup
import os
from glob import glob

package_name = "exploration_benchmarking"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierrebrd",
    maintainer_email="pierrebrd7@gmail.com",
    description="Launch the benchmarking of an exploration algorithm",
    license="MIT",
    entry_points={
        "console_scripts": [
            "tf_filter = exploration_benchmarking.tf_filter_node:main",
            "distance_check = exploration_benchmarking.distance_check:main",
            "laser_scan_check = exploration_benchmarking.laser_scan_check:main",
            "nav_stack_listener = exploration_benchmarking.nav_stack_listener:main",
            "estimated_pose_publisher = exploration_benchmarking.estimated_pose_publisher:main",
        ],
    },
)
