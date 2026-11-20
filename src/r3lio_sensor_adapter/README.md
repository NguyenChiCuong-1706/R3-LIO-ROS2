# Raw Livox adapter for R3LIO

This independent ROS 2 Kilted node converts the recorded input interfaces without changing R3LIO. It subscribes to `/livox/lidar` (`livox_ros_driver2/msg/CustomMsg`), `/imu/data_raw` (`sensor_msgs/msg/Imu`), and `/joint_states` (`sensor_msgs/msg/JointState`, joint `lidar_joint`). It publishes `/ZG/lidar` (`sensor_msgs/msg/PointCloud2`), `/ZG/IMU_data` (unchanged `Imu`), and `/ZG/Motor` (`rigelslam_rot/msg/Motor`). It has no `/calibrated_livox` subscription.

For this workspace, source `/opt/ros/kilted/setup.bash`, `/home/cuong/ws_livox/install/setup.bash`, and this worktree's `install/setup.bash` in that order. Build with `colcon build --packages-select r3lio_sensor_adapter --executor sequential --parallel-workers 1`. Start the adapter alone with `ros2 launch r3lio_sensor_adapter bag_adapter.launch.py`. The interface check is `python3 src/r3lio_sensor_adapter/test/test_adapter_interface.py` with a writable `ROS_LOG_DIR` if needed.

Each raw point keeps its `x,y,z` values and order. Livox `reflectivity` and `line` are widened exactly to `uint16` `intensity` and `ring`. The PointCloud2 `time:float64` value is seconds relative to its unchanged header timestamp, reconstructed from Livox `timebase` and `offset_time` nanoseconds. IMU messages and their timestamps are forwarded unchanged. `lidar_joint` is selected by name; its unwrapped radian position is reduced modulo one revolution and converted to degrees for the original motor callback. No zero offset, rotation sign correction, or spatial transform is applied.

Do not play the bag into R3LIO odometry until the motor zero, direction, and the correct `equipment_param.txt` extrinsics for this rig are confirmed. The original R3LIO motor compensation must remain active.
