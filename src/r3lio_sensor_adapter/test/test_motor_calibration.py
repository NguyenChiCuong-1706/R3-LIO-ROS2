"""Check the optional motor calibration adapter without starting odometry."""

import math
import os
import subprocess
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from rigelslam_rot.msg import Motor
from sensor_msgs.msg import JointState


def main():
    rclpy.init()
    node = rclpy.create_node('r3lio_motor_calibration_test')
    received = []
    subscriber = node.create_subscription(
        Motor, '/ZG/Motor', received.append, 10)
    publisher = node.create_publisher(
        JointState, '/joint_states', qos_profile_sensor_data)
    executable = os.environ.get('R3LIO_ADAPTER_EXECUTABLE',
                                'install/r3lio_sensor_adapter/lib/r3lio_sensor_adapter/r3lio_sensor_adapter')
    adapter = subprocess.Popen([
        executable, '--ros-args',
        '-p', 'motor_zero_rad:=0.1',
        '-p', 'motor_direction:=-1',
        '-p', 'motor_time_shift_s:=0.125',
    ], env=os.environ.copy())
    try:
        deadline = time.monotonic() + 10
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        assert publisher.get_subscription_count() > 0

        joint = JointState()
        joint.header.stamp.sec = 1763000000
        joint.header.stamp.nanosec = 900000000
        joint.name = ['lidar_joint']
        joint.position = [0.3]
        while not received and time.monotonic() < deadline:
            publisher.publish(joint)
            rclpy.spin_once(node, timeout_sec=0.1)
        assert received
        motor = received[0]
        assert motor.header.stamp.sec == 1763000001
        assert motor.header.stamp.nanosec == 25000000
        expected = math.degrees((-(0.3 - 0.1)) % (2 * math.pi))
        assert abs(motor.motor_angle - expected) < 2e-5
        print('PASS: motor zero, direction, timestamp shift')
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
