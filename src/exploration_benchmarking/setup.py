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
        (
            "share/" + package_name + "/launch",
            glob(os.path.join("launch", "*")),
        ),
        ("share/" + package_name + "/params", glob(os.path.join("params", "*"))),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="pierrebrd",
    maintainer_email="pierrebrd7@gmail.com",
    description="Launch the benchmarking of an exploration algorithm",
    license="MIT",
    entry_points={
        "console_scripts": [],
    },
)
