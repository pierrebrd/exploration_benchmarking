# Exploration benchmarking

This project aims to provide a benchmarking suite for exploration algorithms of indoor robots. It takes as an input an exploration algorithm that subscribes to defined topics, and publishes goals to the navigation stack (nav2). The benchmarking suite launches multiple runs, in different environments (clutter-free, cluttered, various sizes, etc.), and returns the results of the benchmark according to defined metrics, as well as the logs + map snapshots for further analysis of the runs.

- [Installation guide](#installation-guide)
  - [Requirements](#requirements)
- [Usage](#usage)
  - [Exploration algorithm inputs and outputs](#exploration-algorithm-inputs-and-outputs)
  - [Benchmark metrics](#benchmark-metrics)

## Installation guide


### Requirements

- ROS2 Humble
<!-- add other dependencies -->

## Usage


### Exploration algorithm inputs and outputs

A `.launch.py` file is used to launch the benchmark, it launches the simulation + navigation stack, and also launches the chosen exploration algorithm by calling its own launch file. The exploration algorithm has its own parameters managed in its launch file, as well as generic parameters set in the main benchmark launch file.


### Benchmark metrics

<!-- TODO -->