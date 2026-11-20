# R3-LIO ROS 2 Odometry

ROS 2 Kilted port of the R3LIO odometry pipeline for a motorized Livox MID-360 rig. The pipeline uses raw LiDAR points, IMU measurements, and motor encoder angles. A separate ROS 2 adapter converts the recorded sensor messages into the interfaces expected by R3LIO.

## Workspace overview

| Directory | Purpose |
|---|---|
| `src/Rot_SLAM/` | R3LIO odometry algorithm, `r3lio_odometry` executable, configuration, and motor compensation |
| `src/r3lio_sensor_adapter/` | ROS 2 node that converts raw Livox, IMU, and encoder messages |
| `src/r3lio_sensor_adapter/rviz/` | RViz2 configuration for odometry, trajectory, and point clouds |

The calibrated launch file is `src/r3lio_sensor_adapter/launch/calibrated_odometry.launch.py`. It starts both the adapter and R3LIO odometry.

## Sensor interfaces

| Recorded input | Message type | Adapter output |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/msg/CustomMsg` | `/ZG/lidar` (`sensor_msgs/msg/PointCloud2`) |
| `/imu/data_raw` | `sensor_msgs/msg/Imu` | `/ZG/IMU_data` (`sensor_msgs/msg/Imu`) |
| `/joint_states`, joint `lidar_joint` | `sensor_msgs/msg/JointState` | `/ZG/Motor` (`rigelslam_rot/msg/Motor`) |

The adapter preserves raw LiDAR point coordinates and point order. It converts Livox `reflectivity` to `intensity`, `line` to `ring`, and nanosecond point timing to the relative seconds expected by R3LIO. IMU measurements and timestamps are forwarded unchanged. Motor joint positions are converted from radians to degrees.

**Use raw `/livox/lidar` as input.** `/calibrated_livox` is not an input to this pipeline because R3LIO performs its own motor-motion compensation.

## Requirements

- Ubuntu 24.04 and ROS 2 Kilted
- A built and sourced `livox_ros_driver2` ROS 2 overlay
- Build dependencies required by `src/Rot_SLAM/CMakeLists.txt`, including PCL, OpenCV, GTSAM, Ceres **2.1**, LibTorch, and ONNX Runtime
- A ROS 2 bag containing all three input streams listed above

## Build

Replace the Livox overlay path if it differs on your machine:

```bash
cd /path/to/r3lio_ros2_kilted
source /opt/ros/kilted/setup.bash
source /home/cuong/ws_livox/install/setup.bash

colcon build \
  --packages-up-to r3lio_sensor_adapter \
  --executor sequential \
  --parallel-workers 1

source install/setup.bash
```

## Run odometry, bag playback, and RViz2 separately

Open **three terminals**. In every terminal, source the same ROS environment:

```bash
cd /path/to/r3lio_ros2_kilted
source /opt/ros/kilted/setup.bash
source /home/cuong/ws_livox/install/setup.bash
source install/setup.bash
```

**Terminal 1 — R3LIO and sensor adapter**

```bash
ros2 launch r3lio_sensor_adapter calibrated_odometry.launch.py
```

Loop closure feedback is enabled by default. To run the separate configuration without loop closure:

```bash
ros2 launch r3lio_sensor_adapter calibrated_odometry.launch.py loop_closure:=false
```

**Terminal 2 — recorded sensor data**

```bash
ros2 bag play /path/to/your_ros2_bag \
  --clock \
  --rate 1.0 \
  --topics /livox/lidar /imu/data_raw /joint_states
```

**Terminal 3 — RViz2**

```bash
rviz2 -d src/r3lio_sensor_adapter/rviz/r3lio_calibrated.rviz
```

The RViz configuration uses `odom` as its fixed frame and displays the odometry pose, trajectory, and point clouds.

## Check the running topics

```bash
ros2 topic hz /ZG/lidar
ros2 topic hz /ZG/IMU_data
ros2 topic hz /ZG/Motor
ros2 topic hz /rigelslam_rot/lidar/mapping/odometry

ros2 topic echo /rigelslam_rot/lidar/mapping/odometry --once
```

Main visualization topics:

- `/rigelslam_rot/lidar/mapping/odometry`
- `/rigelslam_rot/lidar/mapping/path`
- `/rigelslam_rot/lidar/mapping/cloud_registered`
- `/rigelslam_rot/lidar/deskew/cloud_deskewed`
