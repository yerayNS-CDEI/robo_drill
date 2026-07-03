#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformBroadcaster, TransformException, TransformListener


def yaw_from_quaternion(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def quaternion_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    half_yaw = 0.5 * yaw
    return 0.0, 0.0, math.sin(half_yaw), math.cos(half_yaw)


class TurretFootprintBroadcaster(Node):
    def __init__(self) -> None:
        super().__init__("turret_footprint_broadcaster")

        self.declare_parameter("base_frame", "base_footprint")
        self.declare_parameter("tracked_frame", "turret_link")
        self.declare_parameter("published_frame", "turret_footprint")
        self.declare_parameter("publish_rate_hz", 30.0)
        self.declare_parameter("tf_timeout_s", 0.05)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tracked_frame = str(self.get_parameter("tracked_frame").value)
        self.published_frame = str(self.get_parameter("published_frame").value)
        self.publish_rate_hz = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self._warned_tf = False

        self.timer = self.create_timer(1.0 / self.publish_rate_hz, self._publish_transform)

    def _publish_transform(self) -> None:
        try:
            base_to_tracked = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tracked_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as ex:
            if not self._warned_tf:
                self.get_logger().warn(
                    f"Waiting for TF {self.base_frame} -> {self.tracked_frame}: {ex}"
                )
                self._warned_tf = True
            return

        self._warned_tf = False

        translation = base_to_tracked.transform.translation
        rotation = base_to_tracked.transform.rotation
        yaw = yaw_from_quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)

        projected_x = float(translation.x)
        projected_y = float(translation.y)

        transform = TransformStamped()
        transform.header.stamp = base_to_tracked.header.stamp
        transform.header.frame_id = self.published_frame
        transform.child_frame_id = self.base_frame

        # turret_footprint is the ground-projected turret heading; publish the inverse
        # transform so the existing base_footprint tree remains unchanged below it.
        transform.transform.translation.x = -(cos_yaw * projected_x + sin_yaw * projected_y)
        transform.transform.translation.y = sin_yaw * projected_x - cos_yaw * projected_y
        transform.transform.translation.z = 0.0

        qx, qy, qz, qw = quaternion_from_yaw(-yaw)
        transform.transform.rotation.x = qx
        transform.transform.rotation.y = qy
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TurretFootprintBroadcaster()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
