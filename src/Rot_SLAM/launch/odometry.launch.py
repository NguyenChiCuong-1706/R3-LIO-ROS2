"""Run the original R3LIO odometry path with its loop feedback enabled."""

from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _start(context):
    share = Path(get_package_share_directory("rigelslam_rot"))
    with (share / "config" / "params.yaml").open(encoding="utf-8") as stream:
        original = yaml.safe_load(stream)["rigelslam_rot"]
    params = {f"rigelslam_rot.{key}": value for key, value in original.items()}
    params["rigelslam_rot.detector_config_path"] = str(share / "config" / "LoopConfig.yaml")
    params["rigelslam_rot.equipmentParamsFile"] = str(share / "config" / "equipment_param.txt")
    equipment_file = LaunchConfiguration("equipment_file").perform(context)
    if equipment_file:
        params["rigelslam_rot.equipmentParamsFile"] = equipment_file
    params["rigelslam_rot.loopClosureEnableFlag"] = (
        LaunchConfiguration("loop_closure").perform(context).lower() == "true"
    )
    calibration_file = LaunchConfiguration("calibration_file").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    parameter_sources = [params, {"use_sim_time": use_sim_time}]
    if calibration_file:
        parameter_sources.append(calibration_file)
    return [Node(package="rigelslam_rot", executable="r3lio_odometry",
                 name="ZGSlamRosTest", output="screen", parameters=parameter_sources)]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("loop_closure", default_value="true"),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("equipment_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        OpaqueFunction(function=_start),
    ])
