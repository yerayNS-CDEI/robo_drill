#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from geometry_msgs.msg import PoseWithCovarianceStamped, Pose, Point, Quaternion
import time  # Importar la librería de tiempo


class InitialPosePublisher(Node):

    def __init__(self):
        super().__init__('initial_pose_publisher')
        # Define QoS profile with TRANSIENT_LOCAL
        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )

        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', qos_profile)
        time.sleep(1)  # Delay de 1 segundo
        self.publish_initial_pose()

    def publish_initial_pose(self):
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = "map"
        initial_pose.header.stamp = self.get_clock().now().to_msg()
        initial_pose.pose.pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        self.publisher_.publish(initial_pose)
        self.get_logger().info('Publishing initial pose: "%s"' % initial_pose)


def main(args=None):
    rclpy.init(args=args)
    initial_pose_publisher = InitialPosePublisher()
    rclpy.spin(initial_pose_publisher)  # Mantiene el nodo activo
    initial_pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



