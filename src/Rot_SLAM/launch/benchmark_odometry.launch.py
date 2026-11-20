"""Benchmark configuration: the existing loop closure parameter is false."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_file", default_value=""),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(Path(__file__).with_name("odometry.launch.py"))),
            launch_arguments={"loop_closure": "false",
                              "calibration_file": LaunchConfiguration("calibration_file")}.items(),
        ),
    ])
