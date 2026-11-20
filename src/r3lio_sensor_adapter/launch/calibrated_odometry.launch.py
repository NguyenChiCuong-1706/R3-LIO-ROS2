"""Run raw Livox adapter and R3LIO with user-selected MID-360 VERSION 2 calibration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("rigelslam_rot"))
    adapter_share = Path(get_package_share_directory("r3lio_sensor_adapter"))
    calibration_file = str(share / "config" / "mid360_v2_calibration.yaml")
    equipment_file = str(share / "config" / "mid360_v2_equipment.txt")
    return LaunchDescription([
        DeclareLaunchArgument("loop_closure", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(adapter_share / "launch" / "bag_adapter.launch.py")),
            launch_arguments={"calibration_file": calibration_file,
                              "use_sim_time": "true"}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(share / "launch" / "odometry.launch.py")),
            launch_arguments={"loop_closure": LaunchConfiguration("loop_closure"),
                              "calibration_file": calibration_file,
                              "equipment_file": equipment_file,
                              "use_sim_time": "true"}.items(),
        ),
    ])
