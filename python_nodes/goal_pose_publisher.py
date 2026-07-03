#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import time


class GoalPublisher(Node):

    def __init__(self):
        super().__init__('goal_publisher')
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(PoseStamped, '/goal_pose', qos_profile)

        # Declare parameters
        self.declare_parameter('x', 5.5)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 0.0)
        self.declare_parameter('qx', 0.0)
        self.declare_parameter('qy', 0.0)
        self.declare_parameter('qz', 0.0)
        self.declare_parameter('qw', 1.0)

        time.sleep(1)
        self.publish_goal()

    def publish_goal(self):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()

        # Get parameters
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value
        qx = self.get_parameter('qx').get_parameter_value().double_value
        qy = self.get_parameter('qy').get_parameter_value().double_value
        qz = self.get_parameter('qz').get_parameter_value().double_value
        qw = self.get_parameter('qw').get_parameter_value().double_value

        goal.pose = Pose(
            position=Point(x=x, y=y, z=z),
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw)
        )
        self.publisher_.publish(goal)
        self.get_logger().info('Publishing goal: "%s"' % goal)


def main(args=None):
    rclpy.init(args=args)
    goal_publisher = GoalPublisher()
    rclpy.spin(goal_publisher)
    goal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


