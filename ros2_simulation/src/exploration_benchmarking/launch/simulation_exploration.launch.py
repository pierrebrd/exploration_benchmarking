# Simple launch file to launch a simulation without the benchmarking part.

# DEPRECATED, we don't use this anymore. TODO: remove

from timeit import Timer
from launch import LaunchContext, LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    ExecuteProcess,
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction
from launch.conditions import IfCondition
import datetime


def generate_launch_description():

    # Create the arguments
    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")

    nav2_params_file = LaunchConfiguration("nav2_params_file")
    nav2_params_file_arg = DeclareLaunchArgument(
        "nav2_params_file",
        default_value=os.path.join(
            FindPackageShare("exploration_benchmarking").find(
                "exploration_benchmarking"
            ),
            "params",
            "nav2_params.yaml",
        ),
    )

    exploration_params_file = LaunchConfiguration("exploration_params_file")
    exploration_params_file_arg = DeclareLaunchArgument(
        "exploration_params_file",
        default_value=os.path.join(
            FindPackageShare("explore_lite").find("explore_lite"),
            "config",
            "params.yaml",
        ),
    )

    world = LaunchConfiguration("world")
    world_arg = DeclareLaunchArgument("world", default_value="cave")

    launch_rviz = LaunchConfiguration("launch_rviz")
    launch_rviz_arg = DeclareLaunchArgument(
        "launch_rviz",
        default_value="true",
        description="Launch rviz with the simulation",
    )

    rviz_config = LaunchConfiguration("rviz_config")
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="explore-lite",
    )

    # Launch Stage
    stage_ros2_share = FindPackageShare("stage_ros2").find("stage_ros2")
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stage_ros2_share, "launch", "stage.launch.py")
        ),
        launch_arguments={
            "world": world,
            "enforce_prefixes": "false",
            "use_static_transformations": "true",
            "one_tf_tree": "false",
        }.items(),
    )

    # Launch the SLAM algorithm
    # For now, we use the ros2 port of gmapping
    slam_node = Node(
        package="gmapper",
        executable="gmap",
        name="slam_gmapping",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        arguments=["--ros-args", "--log-level", "info", "--log-level", "rcl:=warn"],
    )

    # Launch Nav2 stack
    # We don't need the full Nav2 stack (map management, slam, etc.), just basic navigation capabilities
    # by listening to the map created by the SLAM algorithm and the goals/paths created by the exploration algorithm
    nav2_bringup_share = FindPackageShare("nav2_bringup").find("nav2_bringup")
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": "",
            "autostart": "true",
            "params_file": nav2_params_file,
            "log_level": "info",
        }.items(),
    )

    # Launch the exploration algorithm
    # Little test using explore_lite, but it would be better to use a custom launch file for the exploration algorithm and passing argumetns to it
    # Explore-lite
    explore_node = Node(
        package="explore_lite",
        name="explore_node",
        executable="explore",
        output="screen",
        parameters=[
            exploration_params_file,
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
        arguments=["--ros-args", "--log-level", "info", "--log-level", "rcl:=warn"],
    )

    # Rviz
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                FindPackageShare("exploration_benchmarking").find(
                    "exploration_benchmarking"
                ),
                "launch",
                "basic_rviz.launch.py",
            )
        ),
        condition=IfCondition(launch_rviz),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "config": rviz_config,
        }.items(),
    )

    # Return the LaunchDescription
    # To ensure the order of the simulation nodes, we use a 2s timer between each launch

    args_list = [
        use_sim_time_arg,
        nav2_params_file_arg,
        exploration_params_file_arg,
        world_arg,
        launch_rviz_arg,
        rviz_config_arg,
    ]
    launch_list_ordered = [
        stage_launch,
        slam_node,
        nav2_navigation_launch,
        explore_node,
    ]
    delayed_launch_list = [
        TimerAction(period=2.0 * index, actions=[action])
        for index, action in enumerate(launch_list_ordered)
    ]
    return LaunchDescription(args_list + [rviz_launch] + delayed_launch_list)
