"""Interface check for the external adapter; never starts R3LIO odometry."""

import math
import os
import struct
import subprocess
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.qos import qos_profile_sensor_data
from rigelslam_rot.msg import Motor
from sensor_msgs.msg import Imu, JointState, PointCloud2


def stamp(message, sec, nanosec):
    message.header.stamp.sec = sec
    message.header.stamp.nanosec = nanosec


def main():
    rclpy.init()
    node = rclpy.create_node('r3lio_adapter_interface_test')
    outputs = {}
    subs = [
        node.create_subscription(PointCloud2, '/ZG/lidar',
                                 lambda msg: outputs.setdefault('lidar', msg), 10),
        node.create_subscription(Imu, '/ZG/IMU_data',
                                 lambda msg: outputs.setdefault('imu', msg), 10),
        node.create_subscription(Motor, '/ZG/Motor',
                                 lambda msg: outputs.setdefault('motor', msg), 10),
    ]
    lidar_pub = node.create_publisher(CustomMsg, '/livox/lidar', qos_profile_sensor_data)
    imu_pub = node.create_publisher(Imu, '/imu/data_raw', qos_profile_sensor_data)
    joint_pub = node.create_publisher(JointState, '/joint_states', qos_profile_sensor_data)

    adapter = subprocess.Popen(['ros2', 'run', 'r3lio_sensor_adapter',
                                'r3lio_sensor_adapter'], env=os.environ.copy())
    try:
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if (lidar_pub.get_subscription_count() and imu_pub.get_subscription_count()
                    and joint_pub.get_subscription_count()):
                break
        else:
            raise AssertionError('Adapter input subscriptions were not discovered')

        lidar = CustomMsg()
        stamp(lidar, 1763000000, 250000000)
        lidar.header.frame_id = 'livox_frame'
        lidar.timebase = 1763000000250000000
        lidar.point_num = 2
        p1 = CustomPoint()
        p1.offset_time = 0
        p1.x, p1.y, p1.z = 1.25, -2.5, 3.75
        p1.reflectivity, p1.line = 255, 3
        p2 = CustomPoint()
        p2.offset_time = 100264715
        p2.x, p2.y, p2.z = -4.5, 5.0, 6.25
        p2.reflectivity, p2.line = 9, 0
        lidar.points = [p1, p2]

        imu = Imu()
        stamp(imu, 1763000000, 260000000)
        imu.header.frame_id = 'imu_link'
        imu.angular_velocity.z = 0.0123
        imu.linear_acceleration.z = 9.81
        joint = JointState()
        stamp(joint, 1763000000, 270000000)
        joint.name = ['other_joint', 'lidar_joint']
        joint.position = [1.0, 150.6919]

        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and len(outputs) < 3:
            lidar_pub.publish(lidar)
            imu_pub.publish(imu)
            joint_pub.publish(joint)
            rclpy.spin_once(node, timeout_sec=0.2)
        assert len(outputs) == 3, f'Missing outputs: {outputs.keys()}'

        cloud = outputs['lidar']
        assert cloud.header == lidar.header
        assert cloud.height == 1 and cloud.width == 2
        assert cloud.point_step == 24 and cloud.row_step == 48
        assert [(f.name, f.offset, f.datatype) for f in cloud.fields] == [
            ('x', 0, 7), ('y', 4, 7), ('z', 8, 7),
            ('intensity', 12, 4), ('ring', 14, 4), ('time', 16, 8)]
        first = struct.unpack_from('<fffHHd', cloud.data, 0)
        second = struct.unpack_from('<fffHHd', cloud.data, 24)
        assert first == (1.25, -2.5, 3.75, 255, 3, 0.0), first
        assert second[:5] == (-4.5, 5.0, 6.25, 9, 0), second
        assert abs(second[5] - 0.100264715) < 1e-12, second
        assert outputs['imu'] == imu
        motor = outputs['motor']
        assert motor.header == joint.header
        expected = math.degrees(150.6919 % (2 * math.pi))
        assert abs(motor.motor_angle - expected) < 2e-5, (motor.motor_angle, expected)
        print('PASS: source topics, field layout, coordinates, timestamps, IMU, motor units')
    finally:
        adapter.terminate()
        try:
            adapter.wait(timeout=5)
        except subprocess.TimeoutExpired:
            adapter.kill()
            adapter.wait()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
