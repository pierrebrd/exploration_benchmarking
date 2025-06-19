# Simple launch file to launch a simulation without the benchmarking part.

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

    # TODO: put that in a a param file?

    # TODO: add rviz parameters

    # Launch Stage
    stage_ros2_share = FindPackageShare("stage_ros2").find("stage_ros2")
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stage_ros2_share, "launch", "stage.launch.py")
        ),
        launch_arguments={
            "world": "cave",
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
        remappings=[("/scan", "/base_scan")],
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

    # Return the LaunchDescription
    # To ensure the order of the nodes, we use a 2s timer between each launch

    args_list = [
        use_sim_time_arg,
        nav2_params_file_arg,
        exploration_params_file_arg,
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
    return LaunchDescription(args_list + delayed_launch_list)
