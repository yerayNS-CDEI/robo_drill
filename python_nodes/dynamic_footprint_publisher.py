#!/usr/bin/env python3

import math
from typing import Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point32, Polygon
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener


Point2D = Tuple[float, float]


class DynamicFootprintPublisher(Node):
    def __init__(self) -> None:
        super().__init__("dynamic_footprint_publisher")

        # Topics
        self.declare_parameter("local_footprint_topic", "local_costmap/footprint")
        self.declare_parameter("global_footprint_topic", "global_costmap/footprint")
        self.declare_parameter("joint_states_topic", "/joint_states")

        # Publish behavior
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("joint_state_timeout_s", 1.0)
        self.declare_parameter("circle_samples", 24)

        # Base footprint envelope
        self.declare_parameter("base_radius", 0.7)

        # Arm model (UR10-style approximation in the turret_link frame)
        self.declare_parameter("enable_arm_expansion", True)
        self.declare_parameter("use_tf_for_arm_tip", True)
        self.declare_parameter("robot_base_frame", "turret_link")
        self.declare_parameter("arm_tool_frame", "arm_tool0")
        self.declare_parameter("tf_timeout_s", 0.10)
        self.declare_parameter("arm_direction_yaw_offset", 0.0)
        self.declare_parameter("arm_shoulder_pan_joint", "arm_shoulder_pan_joint")
        self.declare_parameter("arm_shoulder_lift_joint", "arm_shoulder_lift_joint")
        self.declare_parameter("arm_elbow_joint", "arm_elbow_joint")
        self.declare_parameter("arm_mount_x", 0.0)
        self.declare_parameter("arm_mount_y", 0.0)
        self.declare_parameter("arm_mount_yaw", -2.3562)
        self.declare_parameter("upper_arm_length", 0.613)
        self.declare_parameter("forearm_length", 0.572)
        self.declare_parameter("tool_padding", 0.35)
        self.declare_parameter("arm_tip_radius", 0.30)
        self.declare_parameter("max_arm_reach", 1.8)
        self.declare_parameter("reach_alpha", 1.0)

        self.local_footprint_topic = str(self.get_parameter("local_footprint_topic").value)
        self.global_footprint_topic = str(self.get_parameter("global_footprint_topic").value)
        self.joint_states_topic = str(self.get_parameter("joint_states_topic").value)

        self.publish_rate_hz = float(self.get_parameter("publish_rate_hz").value)
        self.joint_state_timeout_s = float(self.get_parameter("joint_state_timeout_s").value)
        self.circle_samples = max(8, int(self.get_parameter("circle_samples").value))

        self.base_radius = float(self.get_parameter("base_radius").value)

        self.enable_arm_expansion = bool(self.get_parameter("enable_arm_expansion").value)
        self.use_tf_for_arm_tip = bool(self.get_parameter("use_tf_for_arm_tip").value)
        self.robot_base_frame = str(self.get_parameter("robot_base_frame").value)
        self.arm_tool_frame = str(self.get_parameter("arm_tool_frame").value)
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)
        self.arm_direction_yaw_offset = float(self.get_parameter("arm_direction_yaw_offset").value)
        self.arm_shoulder_pan_joint = str(self.get_parameter("arm_shoulder_pan_joint").value)
        self.arm_shoulder_lift_joint = str(self.get_parameter("arm_shoulder_lift_joint").value)
        self.arm_elbow_joint = str(self.get_parameter("arm_elbow_joint").value)
        self.arm_mount_x = float(self.get_parameter("arm_mount_x").value)
        self.arm_mount_y = float(self.get_parameter("arm_mount_y").value)
        self.arm_mount_yaw = float(self.get_parameter("arm_mount_yaw").value)
        self.upper_arm_length = float(self.get_parameter("upper_arm_length").value)
        self.forearm_length = float(self.get_parameter("forearm_length").value)
        self.tool_padding = float(self.get_parameter("tool_padding").value)
        self.arm_tip_radius = float(self.get_parameter("arm_tip_radius").value)
        self.max_arm_reach = float(self.get_parameter("max_arm_reach").value)
        self.reach_alpha = float(self.get_parameter("reach_alpha").value)

        self._joint_positions: Dict[str, float] = {}
        self._last_joint_rx_s: Optional[float] = None
        self._smoothed_reach: Optional[float] = None
        self._warned_missing_joints = False
        self._warned_tf_lookup = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.local_pub = self.create_publisher(Polygon, self.local_footprint_topic, 10)
        self.global_pub = self.create_publisher(Polygon, self.global_footprint_topic, 10)
        self.joint_sub = self.create_subscription(
            JointState, self.joint_states_topic, self._on_joint_states, 10
        )

        period = 1.0 / max(self.publish_rate_hz, 1e-3)
        self.timer = self.create_timer(period, self._publish_footprint)

        self.get_logger().info(
            "Dynamic footprint publisher started "
            f"(base_radius={self.base_radius:.2f}, local_topic={self.local_footprint_topic}, "
            f"global_topic={self.global_footprint_topic})"
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_joint_states(self, msg: JointState) -> None:
        for idx, name in enumerate(msg.name):
            if idx < len(msg.position):
                self._joint_positions[name] = float(msg.position[idx])
        self._last_joint_rx_s = self._now_s()

    def _get_joint(self, name: str) -> Optional[float]:
        return self._joint_positions.get(name, None)

    def _sample_circle(self, cx: float, cy: float, radius: float, samples: int) -> List[Point2D]:
        out: List[Point2D] = []
        for i in range(samples):
            a = (2.0 * math.pi * i) / float(samples)
            out.append((cx + radius * math.cos(a), cy + radius * math.sin(a)))
        return out

    @staticmethod
    def _cross(o: Point2D, a: Point2D, b: Point2D) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])

    def _convex_hull(self, points: List[Point2D]) -> List[Point2D]:
        pts = sorted(set(points))
        if len(pts) <= 3:
            return pts

        lower: List[Point2D] = []
        for p in pts:
            while len(lower) >= 2 and self._cross(lower[-2], lower[-1], p) <= 0.0:
                lower.pop()
            lower.append(p)

        upper: List[Point2D] = []
        for p in reversed(pts):
            while len(upper) >= 2 and self._cross(upper[-2], upper[-1], p) <= 0.0:
                upper.pop()
            upper.append(p)

        return lower[:-1] + upper[:-1]

    def _compute_arm_tip(self) -> Optional[Tuple[float, float, float]]:
        if not self.enable_arm_expansion:
            return None

        if self.use_tf_for_arm_tip:
            tip_from_tf = self._compute_arm_tip_from_tf()
            if tip_from_tf is not None:
                return tip_from_tf

        return self._compute_arm_tip_from_joint_model()

    def _compute_arm_tip_from_tf(self) -> Optional[Tuple[float, float, float]]:
        try:
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.arm_tool_frame,
                Time(),
                timeout=Duration(seconds=self.tf_timeout_s),
            )
        except TransformException as ex:
            if not self._warned_tf_lookup:
                self.get_logger().warn(
                    f"TF lookup failed for {self.robot_base_frame} -> {self.arm_tool_frame}: {ex}. "
                    "Falling back to joint-model arm footprint expansion."
                )
                self._warned_tf_lookup = True
            return None

        self._warned_tf_lookup = False
        tip_x = float(transform.transform.translation.x)
        tip_y = float(transform.transform.translation.y)

        dx = tip_x - self.arm_mount_x
        dy = tip_y - self.arm_mount_y
        if abs(dx) < 1e-6 and abs(dy) < 1e-6:
            return None

        yaw = math.atan2(dy, dx) + self.arm_direction_yaw_offset
        tip_x = tip_x + self.tool_padding * math.cos(yaw)
        tip_y = tip_y + self.tool_padding * math.sin(yaw)
        return tip_x, tip_y, yaw

    def _compute_arm_tip_from_joint_model(self) -> Optional[Tuple[float, float, float]]:
        now_s = self._now_s()
        if self._last_joint_rx_s is None or (now_s - self._last_joint_rx_s) > self.joint_state_timeout_s:
            return None

        q_pan = self._get_joint(self.arm_shoulder_pan_joint)
        q_lift = self._get_joint(self.arm_shoulder_lift_joint)
        q_elbow = self._get_joint(self.arm_elbow_joint)
        if q_pan is None or q_lift is None or q_elbow is None:
            if not self._warned_missing_joints:
                self.get_logger().warn(
                    "Missing arm joints in /joint_states. "
                    f"Expected: {self.arm_shoulder_pan_joint}, "
                    f"{self.arm_shoulder_lift_joint}, {self.arm_elbow_joint}. "
                    "Publishing base circular footprint only."
                )
                self._warned_missing_joints = True
            return None

        # Planar reach approximation using shoulder and elbow pitch.
        planar_reach = abs(
            self.upper_arm_length * math.cos(q_lift)
            + self.forearm_length * math.cos(q_lift + q_elbow)
        )
        raw_reach = min(self.max_arm_reach, max(0.0, planar_reach + self.tool_padding))

        if self._smoothed_reach is None:
            self._smoothed_reach = raw_reach
        else:
            alpha = min(1.0, max(0.0, self.reach_alpha))
            self._smoothed_reach = (1.0 - alpha) * self._smoothed_reach + alpha * raw_reach

        yaw = self.arm_mount_yaw + q_pan
        tip_x = self.arm_mount_x + self._smoothed_reach * math.cos(yaw)
        tip_y = self.arm_mount_y + self._smoothed_reach * math.sin(yaw)
        return tip_x, tip_y, yaw

    def _build_polygon_points(self) -> List[Point2D]:
        base_pts = self._sample_circle(0.0, 0.0, self.base_radius, self.circle_samples)
        arm_tip = self._compute_arm_tip()
        if arm_tip is None:
            return base_pts

        tip_x, tip_y, _ = arm_tip
        arm_pts = self._sample_circle(tip_x, tip_y, self.arm_tip_radius, self.circle_samples)
        hull = self._convex_hull(base_pts + arm_pts)
        if len(hull) < 3:
            return base_pts
        return hull

    def _publish_footprint(self) -> None:
        polygon_points = self._build_polygon_points()
        msg = Polygon()
        for x, y in polygon_points:
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            msg.points.append(p)

        self.local_pub.publish(msg)
        self.global_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DynamicFootprintPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
