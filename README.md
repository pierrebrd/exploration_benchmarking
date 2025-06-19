# Exploration benchmarking

This project aims to provide a benchmarking suite for exploration algorithms of indoor robots. It takes as an input an exploration algorithm that subscribes to defined topics, and publishes goals to the navigation stack (nav2). The benchmarking suite launches multiple runs, in different environments (clutter-free, cluttered, various sizes, etc.), and returns the results of the benchmark according to defined metrics, as well as the logs + map snapshots for further analysis of the runs.

This project's root folder is a ROS2 workspace that contains the packages and launch files needed to run the simulations, and the folder also contains python scripts to run the benchmarks and analyze the results.

- [Installation guide](#installation-guide)
  - [Requirements](#requirements)
  - [Cloning the project](#cloning-the-project)
  - [Installing dependencies](#installing-dependencies)
  - [Building the package](#building-the-package)
- [Usage](#usage)
  - [Simulation](#simulation)
  - [Exploration algorithm inputs and outputs](#exploration-algorithm-inputs-and-outputs)
  - [Benchmarking](#benchmarking)
    - [Benchmark metrics](#benchmark-metrics)

## Installation guide


### Requirements

- ROS2 Humble on Ubuntu 22.04 Jammy
<!-- TODO add other dependencies -->

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

<!-- TODO: not sure that building stage alone first is necessary -->

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



### Simulation

The `simulation_exploration.launch.py` launch file is used to launch the simulation, it launches the simulation + navigation stack, and also launches the chosen exploration algorithm.

To launch the simulation and the recording of the rosbag and maps, run in the ros2_simulation folder:

```bash
source install/setup.bash
python3 singlerun.py
```



Future: different launch file for the exploration part. The exploration algorithm should have its own parameters managed in its launch file, as well as generic parameters set in the main launch file.

### Exploration algorithm inputs and outputs

The exploration algorithm should subscribe to the following topics:
- `/map` (nav_msgs/msg/OccupancyGrid): the map of the environment
- `explore/resume` (std_msgs/msg/Bool): a boolean to stop the exploration (`False`) or to resume it (`True`)
  
Those topics are optional:
- `map_updates` (nav_msgs/msg/OccupancyGrid): the map updates
- `/clock` (rosgraph_msgs/msg/Clock): clock topic used to synchronize nodes when using simulation time to speed up the simulation

The exploration algorithm should publish the following topics:
- `goal_sent` (geometry_msgs/msg/Point), when a goal is sent to the navigation stack
- `goal_reached` (geometry_msgs/msg/Point), when a goal is reached by the robot, or when the exploration is stopped manually or automatically (in this case, the Point is (0, 0, 0))
<!-- TODO: use a different method that goal_reached to indicate the end of the simulation -->


### Benchmarking

After the simulation is finished, a benchmarking script analyze the results (rosbags, maps, etc.) and returns the results according to defined metrics. 

The benchmarking script `benchmark.py` is a simple python script, not a ROS2 node. Launch it in a terminal:

```bash
python3 benchmark.py relative/path/to/rosbag2/folder [--verbose]
```

#### Benchmark metrics

Planned metrics:
- Time to obtain a complete map
- Time to obtain a sufficiently complete map (e.g. using a criterion similar to the one presented [here](https://aislabunimi.github.io/explore-stop/))
- Other ideas: difference with ground truth (deformation cost?), back and forth movements, performance in cluttered environments, multirobot exploration, etc.

<!-- TODO -->