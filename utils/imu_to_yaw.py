#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import argparse
from tf_transformations import euler_from_quaternion

class IMUToYaw(Node):
    def __init__(self, topic_name):
        super().__init__('imu_to_yaw_node')
        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            self.imu_callback,
            10)
        self.get_logger().info(f"Subscribed to IMU topic: {topic_name}")

    def imu_callback(self, msg):
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        _, _, yaw = euler_from_quaternion(quaternion)  # Roll, Pitch, Yaw
        yaw_degrees = np.degrees(yaw)
        self.get_logger().info(f"Yaw: {yaw_degrees:.2f} degrees")

def main(args=None):
    rclpy.init(args=args)

    parser = argparse.ArgumentParser(description="IMU to Yaw Node")
    parser.add_argument('--topic', type=str, default='/imu', help="IMU topic name")
    parsed_args, _ = parser.parse_known_args()

    node = IMUToYaw(parsed_args.topic)
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
