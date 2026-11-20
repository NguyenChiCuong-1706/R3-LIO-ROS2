"""Adapt the recorded raw Livox, IMU and lidar_joint streams for R3LIO."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    parameters = [calibration_file] if calibration_file else []
    parameters.append({"use_sim_time": use_sim_time})
    return [Node(package="r3lio_sensor_adapter", executable="r3lio_sensor_adapter",
                 name="r3lio_sensor_adapter", output="screen", parameters=parameters)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_start),
    ])
