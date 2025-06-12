# Simple launch file to launch a simulation without the benchmarking part.
# TODO : try it ;)

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare, FindPackagePrefix
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from launch.actions import TimerAction


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

    # Launch Stage
    stage_ros2_share = FindPackageShare("stage_ros2")
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

    # Nav2 stack
    # We don't need the full Nav2 stack (map management, slam, etc.), just basic navigation capabilities
    # by listening to the map created by the SLAM algorithm and the goals/paths created by the exploration algorithm
    nav2_bringup_share = FindPackageShare("nav2_bringup")
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "namespace": "",
            "autostart": "true",
            "params_file": nav2_params_file,
        }.items(),
    )

    # Return the LaunchDescription
    return LaunchDescription([use_sim_time_arg, stage_launch])
