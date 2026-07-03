#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from std_msgs.msg import Float64MultiArray


class TurretJoy(Node):

    def __init__(self):
        super().__init__('turret_joy_node')
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            1)
        self.get_logger().info('Instantiated Turret Joy node"')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            'turret_controller/commands',
            1)
        self.speed = 0.15  # Default speed
        self.sent_disable_msg = False

        self.min_speed = self.declare_parameter('min_turret_speed', 0.01).get_parameter_value().double_value
        self.max_speed = self.declare_parameter('max_turret_speed', 0.05).get_parameter_value().double_value
        self.subscription  # prevent unused variable warning

    def joy_callback(self, msg):
        # Assuming buttons field in msg represents the array of buttons
        axes = msg.axes
        if abs(axes[4]):
            # Publish turret velocity
            command_msg = Float64MultiArray()
            command_msg.data = [self.speed * axes[4]]  # Calculate and set speed
            self.publisher.publish(command_msg)
            self.sent_disable_msg = False

        elif not axes[4]:
            if not self.sent_disable_msg:
                command_msg = Float64MultiArray()
                command_msg.data = [0.0]  # Calculate and set speed
                self.publisher.publish(command_msg)
                self.sent_disable_msg = True

        if axes[5] > 0.001:
            self.speed += 0.05
            self.speed = max(self.min_speed, min(self.max_speed, self.speed))
        if axes[5] < -0.001:
            self.speed -= 0.05
            self.speed = max(self.min_speed, min(self.max_speed, self.speed))

def main(args=None):
    rclpy.init(args=args)

    turret_joy_subscriber = TurretJoy()

    rclpy.spin(turret_joy_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    turret_joy_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()