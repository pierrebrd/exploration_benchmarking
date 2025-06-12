# Exploration benchmarking

This project aims to provide a benchmarking suite for exploration algorithms of indoor robots. It takes as an input an exploration algorithm that subscribes to defined topics, and publishes goals to the navigation stack (nav2). The benchmarking suite launches multiple runs, in different environments (clutter-free, cluttered, various sizes, etc.), and returns the results of the benchmark according to defined metrics, as well as the logs + map snapshots for further analysis of the runs.

- [Installation guide](#installation-guide)
  - [Requirements](#requirements)
  - [Cloning the project](#cloning-the-project)
  - [Installing dependencies](#installing-dependencies)
  - [Building the package](#building-the-package)
- [Usage](#usage)
  - [Exploration algorithm inputs and outputs](#exploration-algorithm-inputs-and-outputs)
  - [Benchmark metrics](#benchmark-metrics)

## Installation guide


### Requirements

- ROS2 Humble
<!-- add other dependencies -->

### Cloning the project

```bash
git clone --recurse-submodules https://github.com/aislabunimi/exploration_benchmarking.git
cd exploration_benchmarking
```

### Installing dependencies

```bash
sudo apt update
sudo apt install python3-colcon-common-extensions python3-rosdep
# Stage requirements
sudo apt-get install git cmake g++ libjpeg8-dev libpng-dev libglu1-mesa-dev libltdl-dev libfltk1.1-dev 
# Install ROS2 dependencies
rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```


### Building the package

1. First, build the Stage package only and source it:
   ```bash
   colcon build --symlink-install --packages-select stage
   source install/setup.bash
   ```
2. Then, build the other packages:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

<!-- colcon build --symlink-install --cmake-args -DOpenGL_GL_PREFERENCE=LEGACY # Not needed anymore-->


## Usage


### Exploration algorithm inputs and outputs

A `.launch.py` file is used to launch the benchmark, it launches the simulation + navigation stack, and also launches the chosen exploration algorithm by calling its own launch file. The exploration algorithm has its own parameters managed in its launch file, as well as generic parameters set in the main benchmark launch file.


### Benchmark metrics

<!-- TODO -->