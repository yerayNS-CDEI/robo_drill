#!/usr/bin/env python3
import math
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose

from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_pose_stamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from arm_control.srv import ComputeBasePlacement, SendPosition

from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as DurationMsg
from std_msgs.msg import Bool, ColorRGBA, Float64MultiArray

try:
    from arm_control.srv import OptimalBase
except Exception:
    OptimalBase = None
try:
    from tf_transformations import euler_from_quaternion
except Exception:
    euler_from_quaternion = None


class State(Enum):
    IDLE = 0
    WAITING_ARM_POSITION = 10
    WAITING_COLUMN = 1
    WAITING_BASE_SERVICE = 2
    WAITING_NAV2 = 3
    WAITING_NAV2_SETTLE = 4


class GoalRouter(Node):
    def __init__(self):
        super().__init__("goal_router")

        # -----------------------------
        # Parameters
        # -----------------------------
        self.declare_parameter("global_frame", "map")  # global == map
        self.declare_parameter("arm_base_frame", "arm_base")
        self.declare_parameter("robot_base_frame", "base_link")  # frame Nav2 steers
        self.declare_parameter("reach_radius", 1.2)          # meters
        self.declare_parameter("reach_radius_post_nav2_margin_m", 0.4)  # allow small post-Nav2 slack
        self.declare_parameter("tf_timeout_s", 0.5)

        self.declare_parameter("input_goal_topic", "/map_goal_pose")
        self.declare_parameter("arm_goal_topic", "/arm/goal_pose")
        self.declare_parameter("compute_base_placement_srv", "/compute_base_placement")
        self.declare_parameter("compute_optimal_base_srv", "/compute_optimal_base")
        self.declare_parameter("nav2_action_name", "/navigate_to_pose")
        self.declare_parameter("sim", False)
        self.declare_parameter("tf_settle_after_nav2_s", 0.8)
        
        self.declare_parameter("tf_fresh_max_age_s", 0.20)
        self.declare_parameter("tf_stable_pos_eps_m", 0.002)
        self.declare_parameter("tf_stable_yaw_eps_deg", 0.15)
        self.declare_parameter("tf_stable_required_s", 0.20)

        self.declare_parameter("column_vel_tol", 0.002)          # m/s (joint velocity)
        self.declare_parameter("column_settle_time_s", 0.25)     # seconds stable
        
        # Column / vertical reach
        self.declare_parameter("column_admissible_heights", [0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9])
        self.declare_parameter("column_min_height_m", 0.0)
        self.declare_parameter("column_max_height_m", 0.9)
        self.declare_parameter("column_current_height", 0.0)  # update externally as the real actuator moves
        self.declare_parameter("arm_reachable_z_min", 0.0)
        self.declare_parameter("arm_reachable_z_max", 1.1)
        self.declare_parameter("column_tolerance_m", 0.005)   # 5mm
        self.declare_parameter("column_retract_tolerance_m", 0.02)  # looser tolerance for pre-base safety retract
        self.declare_parameter("column_wait_timeout_s", 5.0)
        self.declare_parameter("column_joint_name", "column_joint")
        self.declare_parameter("column_action_name", "/column_controller/follow_joint_trajectory")
        self.declare_parameter("column_command_topic", "/column_position_controller/commands")
        self.declare_parameter("column_move_time_s", 3.0)
        
        # Base
        self.declare_parameter("base_lin_vel_tol", 0.01)   # m/s
        self.declare_parameter("base_ang_vel_tol", 0.02)   # rad/s
        self.declare_parameter("base_settle_time_s", 0.25) # s
        self.declare_parameter("odom_fresh_max_age_s", 0.20)
        
        # Arm staged poses via position_sender_node
        self.declare_parameter("arm_fold_enable", True)
        self.declare_parameter("arm_unfold_enable", True)
        self.declare_parameter("arm_send_position_service_names", ["/arm/send_position", "/send_position"])
        self.declare_parameter("arm_fold_position_name", "folded")
        self.declare_parameter("arm_unfold_position_name", "folded")
        self.declare_parameter("arm_fold_pose", [-0.44101, 0.38792, 0.13674, -0.327, 0.753, 0.564, -0.087])
        self.declare_parameter("arm_unfold_pose", [-0.44101, 0.38792, 0.13674, -0.327, 0.753, 0.564, -0.087])
        self.declare_parameter("arm_named_pose_tool_frames", ["arm_tool0", "arm_tool0_controller"])
        self.declare_parameter("arm_named_pose_pos_tol_m", 0.03)
        self.declare_parameter("arm_named_pose_ang_tol_deg", 10.0)
        self.declare_parameter("arm_named_pose_settle_time_s", 0.25)
        self.declare_parameter("arm_unfold_timeout_s", 60.0)

        # Deprecated legacy params kept for launch/config compatibility.
        self.declare_parameter("arm_action_name", "/joint_trajectory_controller/follow_joint_trajectory")
        self.declare_parameter(
            "arm_fold_joint_names",
            [
                "arm_shoulder_pan_joint",
                "arm_shoulder_lift_joint",
                "arm_elbow_joint",
                "arm_wrist_1_joint",
                "arm_wrist_2_joint",
                "arm_wrist_3_joint",
            ],
        )
        self.declare_parameter("arm_fold_positions", [3.1313, -1.6422, -2.4464, -2.7985, -0.6618, 0.2421])
        self.declare_parameter("arm_fold_move_time_s", 3.0)
        self.declare_parameter("arm_fold_pos_tol_rad", 0.05)
        self.declare_parameter("arm_fold_vel_tol_rad_s", 0.05)
        self.declare_parameter("arm_fold_settle_time_s", 0.25)
        self.declare_parameter("arm_fold_timeout_s", 60.0)

        self.global_frame = self.get_parameter("global_frame").value
        self.arm_base_frame = self.get_parameter("arm_base_frame").value
        self.robot_base_frame = self.get_parameter("robot_base_frame").value
        self.reach_radius = float(self.get_parameter("reach_radius").value)
        self.reach_radius_post_nav2_margin_m = float(
            self.get_parameter("reach_radius_post_nav2_margin_m").value
        )
        self.tf_timeout_s = float(self.get_parameter("tf_timeout_s").value)

        self.input_goal_topic = self.get_parameter("input_goal_topic").value
        self.arm_goal_topic = self.get_parameter("arm_goal_topic").value
        self.compute_base_placement_srv = self.get_parameter("compute_base_placement_srv").value
        self.compute_optimal_base_srv = self.get_parameter("compute_optimal_base_srv").value
        self.sim = self._param_as_bool(self.get_parameter("sim").value)
        self.column_action_name = self.get_parameter("column_action_name").value
        self.column_command_topic = self.get_parameter("column_command_topic").value
        self.nav2_action_name = self.get_parameter("nav2_action_name").value
        self.tf_settle_after_nav2_s = float(self.get_parameter("tf_settle_after_nav2_s").value)
        
        self.tf_fresh_max_age_s = float(self.get_parameter("tf_fresh_max_age_s").value)
        self.tf_stable_pos_eps_m = float(self.get_parameter("tf_stable_pos_eps_m").value)
        self.tf_stable_yaw_eps_rad = math.radians(float(self.get_parameter("tf_stable_yaw_eps_deg").value))
        self.tf_stable_required_s = float(self.get_parameter("tf_stable_required_s").value)

        self.column_vel_tol = float(self.get_parameter("column_vel_tol").value)
        self.column_settle_time_s = float(self.get_parameter("column_settle_time_s").value)
        
        self.column_admissible_heights = [float(v) for v in self.get_parameter("column_admissible_heights").value]
        self.column_min_height_m = float(self.get_parameter("column_min_height_m").value)
        self.column_max_height_m = float(self.get_parameter("column_max_height_m").value)
        if self.column_max_height_m < self.column_min_height_m:
            self.get_logger().warn(
                f"column_max_height_m ({self.column_max_height_m:.3f}) is below column_min_height_m "
                f"({self.column_min_height_m:.3f}); clamping max to min."
            )
            self.column_max_height_m = self.column_min_height_m
        self.arm_reachable_z_min = float(self.get_parameter("arm_reachable_z_min").value)
        self.arm_reachable_z_max = float(self.get_parameter("arm_reachable_z_max").value)
        self.column_tolerance_m = float(self.get_parameter("column_tolerance_m").value)
        self.column_retract_tolerance_m = float(self.get_parameter("column_retract_tolerance_m").value)
        self.column_wait_timeout_s = float(self.get_parameter("column_wait_timeout_s").value)
        self.column_move_time_s = float(self.get_parameter("column_move_time_s").value)
        self.column_joint_name = self.get_parameter("column_joint_name").value
        
        self.base_lin_vel_tol = float(self.get_parameter("base_lin_vel_tol").value)
        self.base_ang_vel_tol = float(self.get_parameter("base_ang_vel_tol").value)
        self.base_settle_time_s = float(self.get_parameter("base_settle_time_s").value)
        self.odom_fresh_max_age_s = float(self.get_parameter("odom_fresh_max_age_s").value)
        
        self.arm_fold_enable = bool(self.get_parameter("arm_fold_enable").value)
        self.arm_unfold_enable = bool(self.get_parameter("arm_unfold_enable").value)
        self.arm_send_position_service_names = [
            str(name) for name in self.get_parameter("arm_send_position_service_names").value
        ]
        self.arm_fold_position_name = str(self.get_parameter("arm_fold_position_name").value)
        self.arm_unfold_position_name = str(self.get_parameter("arm_unfold_position_name").value)
        self.arm_fold_pose = self._pose7_from_param("arm_fold_pose")
        self.arm_unfold_pose = self._pose7_from_param("arm_unfold_pose")
        self.arm_named_pose_tool_frames = [
            str(frame) for frame in self.get_parameter("arm_named_pose_tool_frames").value
        ]
        self.arm_named_pose_pos_tol_m = float(self.get_parameter("arm_named_pose_pos_tol_m").value)
        self.arm_named_pose_ang_tol_rad = math.radians(
            float(self.get_parameter("arm_named_pose_ang_tol_deg").value)
        )
        self.arm_named_pose_settle_time_s = float(
            self.get_parameter("arm_named_pose_settle_time_s").value
        )
        self.arm_unfold_timeout_s = float(self.get_parameter("arm_unfold_timeout_s").value)

        self.arm_action_name = self.get_parameter("arm_action_name").value
        self.arm_fold_joint_names = list(self.get_parameter("arm_fold_joint_names").value)
        self.arm_fold_positions = [float(x) for x in self.get_parameter("arm_fold_positions").value]
        self.arm_fold_move_time_s = float(self.get_parameter("arm_fold_move_time_s").value)
        self.arm_fold_pos_tol_rad = float(self.get_parameter("arm_fold_pos_tol_rad").value)
        self.arm_fold_vel_tol_rad_s = float(self.get_parameter("arm_fold_vel_tol_rad_s").value)
        self.arm_fold_settle_time_s = float(self.get_parameter("arm_fold_settle_time_s").value)
        self.arm_fold_timeout_s = float(self.get_parameter("arm_fold_timeout_s").value)

        self._marker_ns = "goal_router"
        self._marker_id_map = 0
        self._marker_id_arm = 1
        self.column_control_mode = "trajectory_action" if self.sim else "position_topic"
        self.nav2_goal_in_flight = False
        # self.planned_column_delta_h: float | None = None
        self.column_target_height: float | None = None
        self.column_move_deadline = None
        self.column_current_height = None  # updated from /joint_states
        
        self.column_current_velocity = None
        self.last_joint_state_stamp = None
        self._column_stable_since = None
        
        self.last_odom_stamp = None
        self.last_odom_twist = None
        self._base_stable_since = None

        self.nav2_settle_deadline = None
        self._nav2_tf_last = None
        self._nav2_tf_last_yaw = None
        self._nav2_tf_stable_since = None
        self.base_move_done_for_goal = False
        
        # Arm staged-pose tracking
        self.arm_fold_done_for_goal = True
        self.arm_unfold_done_for_goal = True
        self.arm_named_pose_deadline = None
        self._arm_named_pose_stable_since = None
        self._pending_arm_position_name = None
        self._pending_arm_position_pose = None
        self._pending_arm_position_stage = None
        self._arm_position_clients = {}
        self.arm_execution_status = True
        self._arm_execution_started = False
        self._arm_execution_completed = False
        
        # OptimalBase service params (used only if /compute_optimal_base is available)
        self.declare_parameter("optimal_min_dist", 0.2)
        self.declare_parameter("optimal_round_decimals", 3)
        self.declare_parameter("optimal_grid_res", 0.1)
        self.declare_parameter("optimal_x_limits", [-4.0, 4.0])     # TODO: erase if not needed (modification of the srv message then)
        self.declare_parameter("optimal_y_limits", [-4.0, 4.0])     # TODO: erase if not needed (modification of the srv message then)
        self.declare_parameter("optimal_limits_margin", 4.0)
        self.declare_parameter("optimal_enable_simulator", False)
        self.declare_parameter("optimal_enable_robot_viz", False)
        
        self.column_watchdog_timer = self.create_timer(0.05, self._tick)  # 20 Hz
                
        # -----------------------------
        # TF2
        # -----------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # -----------------------------
        # Pub/Sub
        # -----------------------------
        self.goal_sub = self.create_subscription(PoseStamped, self.input_goal_topic, self.on_goal, 10)
        self.odom_sub = self.create_subscription(Odometry, "/rtabmap/odom", self.on_odom, 10)
        self.joint_states_sub = self.create_subscription(JointState, "/joint_states", self.on_joint_states, 10)
        self.arm_execution_status_sub = self.create_subscription(
            Bool, "execution_status", self.on_arm_execution_status, 10
        )
        self.arm_execution_status_sub_global = self.create_subscription(
            Bool, "/execution_status", self.on_arm_execution_status, 10
        )
        self.arm_goal_pub = self.create_publisher(PoseStamped, self.arm_goal_topic, 10)

        self.goal_map_marker_pub = self.create_publisher(Marker, "/goal_router/goal_map_marker", 10)
        self.goal_arm_marker_pub = self.create_publisher(Marker, "/goal_router/goal_arm_marker", 10)

        # -----------------------------
        # Service client
        # -----------------------------
        self.base_srv = self.create_client(ComputeBasePlacement, self.compute_base_placement_srv)
        self.optimal_srv = None
        if OptimalBase is not None:
            self.optimal_srv = self.create_client(OptimalBase, self.compute_optimal_base_srv)
        
        # -----------------------------
        # Column control
        # -----------------------------
        self.column_client = None
        self.column_command_pub = None
        self._configure_column_control()

        # -----------------------------
        # Arm named-position service clients
        # -----------------------------
        self.arm_position_client = None
        self.arm_position_service_name = None

        # -----------------------------
        # Nav2 Action client
        # -----------------------------
        self.nav2_client = ActionClient(self, NavigateToPose, self.nav2_action_name)
        self.nav2_goal_handle = None

        # -----------------------------
        # State
        # -----------------------------
        # Planning context for the currently handled goal
        self.active_goal_map: PoseStamped | None = None
        self.planned_column_height: float | None = None  # selected target height (absolute)
        self.state = State.IDLE

        self.get_logger().info(
            f"GoalRouter ready. in={self.input_goal_topic} arm={self.arm_goal_topic} "
            f"srv={self.compute_base_placement_srv} nav2_action={self.nav2_action_name} "
            f"column_mode={self.column_control_mode} sim={self.sim}"
        )

    def _param_as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _pose7_from_param(self, name: str) -> tuple[float, float, float, float, float, float, float]:
        values = list(self.get_parameter(name).value)
        if len(values) != 7:
            raise ValueError(
                f"Parameter '{name}' must contain 7 values: x, y, z, qx, qy, qz, qw."
            )
        return tuple(float(v) for v in values)

    def _configure_column_control(self):
        if self.column_control_mode == "trajectory_action":
            self.column_client = ActionClient(self, FollowJointTrajectory, self.column_action_name)
            self.column_command_pub = None
            self.get_logger().info(
                f"Column control configured for simulation via {self.column_action_name}"
            )
            return

        self.column_client = None
        self.column_command_pub = self.create_publisher(
            Float64MultiArray, self.column_command_topic, 10
        )
        self.get_logger().info(
            f"Column control configured for hardware via {self.column_command_topic}"
        )

    def _reset_column_command(self):
        self.state = State.IDLE
        self.column_target_height = None
        self.column_move_deadline = None

    def _begin_column_wait(self, target_h: float):
        self.column_target_height = target_h
        self.planned_column_height = target_h
        self.column_move_deadline = self.get_clock().now() + Duration(seconds=self.column_wait_timeout_s)
        self.state = State.WAITING_COLUMN

    # -----------------------------
    # TIMER
    # -----------------------------
    def _tick(self):
        if self.state == State.WAITING_COLUMN:
            if self.column_reached_target():
                self.get_logger().info(f"Column reached target: {self.column_target_height:.3f} m")
                self.state = State.IDLE
                self.column_target_height = None
                self.column_move_deadline = None
                self.plan_and_execute()
                return

            if self.column_move_deadline is not None and self.get_clock().now() > self.column_move_deadline:
                target_h = self.column_target_height
                current_h = self.current_column_height_from_param()
                if target_h is None:
                    self.get_logger().warn(
                        f"Column move timeout at current={current_h:.3f} m. Proceeding with best available TF."
                    )
                else:
                    self.get_logger().warn(
                        f"Column move timeout at current={current_h:.3f} m target={target_h:.3f} m. "
                        "Proceeding with best available TF."
                    )
                    # Snap to target if the column settled close enough but outside column_tolerance_m.
                    # This prevents re-commanding the same target in a loop when the actuator
                    # physically can't close the last few mm.
                    if (self.column_current_height is not None
                            and abs(self.column_current_height - target_h) <= self.column_retract_tolerance_m):
                        self.get_logger().info(
                            f"Snapping column_current_height {self.column_current_height:.4f} -> {target_h:.4f} m "
                            f"(within column_retract_tolerance_m={self.column_retract_tolerance_m:.3f})."
                        )
                        self.column_current_height = target_h
                self.state = State.IDLE
                self.column_target_height = None
                self.column_move_deadline = None
                self.plan_and_execute()
            return

        if self.state == State.WAITING_NAV2_SETTLE:
            if self.base_is_stopped() and self._nav2_tf_is_stable():
                self._finish_after_nav2(note="after Nav2 (TF stable)")
                return

            if self.nav2_settle_deadline is not None and self.get_clock().now() > self.nav2_settle_deadline:
                self.get_logger().warn("TF did not become stable in time; using best available TF.")
                self._finish_after_nav2(note="after Nav2 (TF best-effort)")
            return
        
        if self.state == State.WAITING_ARM_POSITION:
            pose_reached = (
                self._pending_arm_position_pose is not None
                and self.arm_is_at_named_pose(self._pending_arm_position_pose)
            )
            execution_complete = self._arm_execution_started and self._arm_execution_completed

            if execution_complete or pose_reached:
                reached_name = self._pending_arm_position_name or "arm staged"
                if self._pending_arm_position_stage == "fold":
                    self.arm_fold_done_for_goal = True
                    self.arm_unfold_done_for_goal = False
                elif self._pending_arm_position_stage == "unfold":
                    self.arm_unfold_done_for_goal = True

                self.get_logger().info(
                    f"Arm position '{reached_name}' completed. Proceeding with plan."
                )
                self.state = State.IDLE
                self.arm_named_pose_deadline = None
                self._pending_arm_position_name = None
                self._pending_arm_position_pose = None
                self._pending_arm_position_stage = None
                self._arm_execution_started = False
                self._arm_execution_completed = False
                self.plan_and_execute()
                return

            if (
                self.arm_named_pose_deadline is not None
                and self.get_clock().now() > self.arm_named_pose_deadline
            ):
                target_name = self._pending_arm_position_name or "arm staged"
                self.get_logger().error(
                    f"Timed out waiting for arm position '{target_name}'. Aborting current goal."
                )
                self.state = State.IDLE
                self.arm_named_pose_deadline = None
                self._pending_arm_position_name = None
                self._pending_arm_position_pose = None
                self._pending_arm_position_stage = None
                self.arm_fold_done_for_goal = False
                self.arm_unfold_done_for_goal = False
                self._arm_execution_started = False
                self._arm_execution_completed = False
                self._clear_plan()
                return
            return
    
    def on_odom(self, msg: Odometry):
        self.last_odom_stamp = rclpy.time.Time.from_msg(msg.header.stamp)
        self.last_odom_twist = msg.twist.twist

    def on_arm_execution_status(self, msg: Bool):
        self.arm_execution_status = bool(msg.data)

        if self.state != State.WAITING_ARM_POSITION or self._pending_arm_position_name is None:
            return

        if not self.arm_execution_status:
            self._arm_execution_started = True
            self._arm_execution_completed = False
            return

        if self._arm_execution_started:
            self._arm_execution_completed = True
        
    def on_joint_states(self, msg: JointState):
        try:
            idx = msg.name.index(self.column_joint_name)
        except ValueError:
            return
        
        self.last_joint_state_stamp = rclpy.time.Time.from_msg(msg.header.stamp)

        if idx < len(msg.position):
            self.column_current_height = float(msg.position[idx])
            
        if idx < len(msg.velocity):
            self.column_current_velocity = float(msg.velocity[idx])
        else:
            # Some drivers publish position-only JointState messages.
            # Avoid carrying a stale non-zero velocity forever.
            self.column_current_velocity = 0.0

                
    # -----------------------------
    # TF helpers
    # -----------------------------
    def _tf_timeout(self) -> Duration:
        return Duration(seconds=self.tf_timeout_s)

    def arm_base_xy_in_world(self):
        # arm_base pose expressed in world
        tf = self.tf_buffer.lookup_transform(
            self.global_frame, self.arm_base_frame, rclpy.time.Time(), self._tf_timeout()
        )
        return tf.transform.translation.x, tf.transform.translation.y

    def planar_distance_armbase_to_goal(self, goal_world: PoseStamped) -> float:
        ax, ay = self.arm_base_xy_in_world()
        gx = goal_world.pose.position.x
        gy = goal_world.pose.position.y
        return math.hypot(gx - ax, gy - ay)

    def transform_pose_stamped(self, pose_in: PoseStamped, target_frame: str) -> PoseStamped:
        tf = self.tf_buffer.lookup_transform(
            target_frame, pose_in.header.frame_id, rclpy.time.Time(), self._tf_timeout()
        )
        pose_out = do_transform_pose_stamped(pose_in, tf)
        pose_out.header.frame_id = target_frame
        pose_out.header.stamp = self.get_clock().now().to_msg()
        return pose_out

    def transform_goal_to_arm_base(self, goal_world: PoseStamped) -> PoseStamped:
        return self.transform_pose_stamped(goal_world, self.arm_base_frame)
    
    def wait_for_transform(self, target_frame: str, source_frame: str, timeout_s: float) -> bool:
        end_time = self.get_clock().now() + Duration(seconds=timeout_s)
        while self.get_clock().now() < end_time:
            try:
                self.tf_buffer.lookup_transform(
                    target_frame, source_frame, rclpy.time.Time(), self._tf_timeout()
                )
                return True
            except Exception:
                rclpy.spin_once(self, timeout_sec=0.05)
        return False
    
    def _yaw_from_quat(self, q) -> float:
        # yaw around Z from quaternion (no external deps)
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _get_arm_base_pose_in_map(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.arm_base_frame, rclpy.time.Time(), self._tf_timeout()
            )
        except Exception:
            return None

        stamp = rclpy.time.Time.from_msg(tf.header.stamp)
        age = self.get_clock().now() - stamp

        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = self._yaw_from_quat(q)
        return (float(t.x), float(t.y), float(t.z), float(yaw), age)

    def _tf_pose_is_fresh(self) -> bool:
        pose = self._get_arm_base_pose_in_map()
        if pose is None:
            return False
        *_xyz_yaw, age = pose
        return age <= Duration(seconds=self.tf_fresh_max_age_s)

    def _nav2_tf_is_stable(self) -> bool:
        pose = self._get_arm_base_pose_in_map()
        if pose is None:
            self._nav2_tf_stable_since = None
            self._nav2_tf_last = None
            self._nav2_tf_last_yaw = None
            return False

        x, y, z, yaw, age = pose
        if age > Duration(seconds=self.tf_fresh_max_age_s):
            self._nav2_tf_stable_since = None
            self._nav2_tf_last = (x, y, z)
            self._nav2_tf_last_yaw = yaw
            return False

        now = self.get_clock().now()

        if self._nav2_tf_last is None:
            self._nav2_tf_last = (x, y, z)
            self._nav2_tf_last_yaw = yaw
            self._nav2_tf_stable_since = None
            return False

        dx = x - self._nav2_tf_last[0]
        dy = y - self._nav2_tf_last[1]
        dz = z - self._nav2_tf_last[2]
        dp = math.sqrt(dx * dx + dy * dy + dz * dz)

        dyaw = math.atan2(math.sin(yaw - self._nav2_tf_last_yaw), math.cos(yaw - self._nav2_tf_last_yaw))
        dyaw = abs(dyaw)

        self._nav2_tf_last = (x, y, z)
        self._nav2_tf_last_yaw = yaw

        if dp <= self.tf_stable_pos_eps_m and dyaw <= self.tf_stable_yaw_eps_rad:
            if self._nav2_tf_stable_since is None:
                self._nav2_tf_stable_since = now
            return (now - self._nav2_tf_stable_since) >= Duration(seconds=self.tf_stable_required_s)

        self._nav2_tf_stable_since = None
        return False
    
    def _finish_after_nav2(self, note: str):
        self.get_logger().info(note)
        self.state = State.IDLE
        self.nav2_settle_deadline = None
        self.base_move_done_for_goal = True
        # After base is settled, re-run planner:
        # - now planar should be OK
        # - if vertical not OK at column=0.0, it will command column AFTER base
        self.plan_and_execute()
        
    def base_is_stopped(self) -> bool:
        if self.last_odom_stamp is None or self.last_odom_twist is None:
            self._base_stable_since = None
            return False

        now = self.get_clock().now()
        age = now - self.last_odom_stamp
        if age > Duration(seconds=self.odom_fresh_max_age_s):
            self._base_stable_since = None
            return False

        vx = float(self.last_odom_twist.linear.x)
        vy = float(self.last_odom_twist.linear.y)
        wz = float(self.last_odom_twist.angular.z)

        v = math.hypot(vx, vy)
        if v <= self.base_lin_vel_tol and abs(wz) <= self.base_ang_vel_tol:
            if self._base_stable_since is None:
                self._base_stable_since = now
                return False
            return (now - self._base_stable_since) >= Duration(seconds=self.base_settle_time_s)

        self._base_stable_since = None
        return False
        
    # -----------------------------
    # Service selection + helpers
    # -----------------------------
    def _service_is_available(self, srv_name: str) -> bool:
        try:
            names_and_types = self.get_service_names_and_types()
        except Exception:
            return False
        return any(name == srv_name for (name, _types) in names_and_types)

    def _select_base_service(self) -> str | None:
        # Prefer OptimalBase if available and import succeeded
        if self.optimal_srv is not None and self._service_is_available(self.compute_optimal_base_srv):
            return "optimal"
        if self._service_is_available(self.compute_base_placement_srv):
            return "placement"
        return None

    def _get_arm_position_client(self):
        for srv_name in self.arm_send_position_service_names:
            client = self._arm_position_clients.get(srv_name)
            if client is None:
                client = self.create_client(SendPosition, srv_name)
                self._arm_position_clients[srv_name] = client

            if client.wait_for_service(timeout_sec=0.5):
                self.arm_position_client = client
                self.arm_position_service_name = srv_name
                return client, srv_name

        return None, None

    def _quat_to_rpy_deg(self, q) -> tuple[float, float, float]:
        if euler_from_quaternion is None:
            raise RuntimeError("tf_transformations not available (cannot convert quaternion -> RPY).")
        r, p, y = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return math.degrees(r), math.degrees(p), math.degrees(y)

    def _current_arm_tool_pose_in_arm_base(self):
        for tool_frame in self.arm_named_pose_tool_frames:
            try:
                tf = self.tf_buffer.lookup_transform(
                    self.arm_base_frame, tool_frame, rclpy.time.Time(), self._tf_timeout()
                )
            except Exception:
                continue

            pose = PoseStamped()
            pose.header.frame_id = self.arm_base_frame
            pose.header.stamp = tf.header.stamp
            pose.pose.position.x = float(tf.transform.translation.x)
            pose.pose.position.y = float(tf.transform.translation.y)
            pose.pose.position.z = float(tf.transform.translation.z)
            pose.pose.orientation = tf.transform.rotation
            return pose

        return None

    def _quat_angle_error_rad(self, qa, qb) -> float:
        dot = (
            float(qa[0]) * float(qb[0])
            + float(qa[1]) * float(qb[1])
            + float(qa[2]) * float(qb[2])
            + float(qa[3]) * float(qb[3])
        )
        dot = max(-1.0, min(1.0, abs(dot)))
        return 2.0 * math.acos(dot)

    # -----------------------------
    # Markers
    # -----------------------------
    def clear_goal_markers(self):
        """Clear previous goal markers on both map and arm marker topics."""
        now = self.get_clock().now().to_msg()

        clear_map = Marker()
        clear_map.header.frame_id = self.global_frame
        clear_map.header.stamp = now
        clear_map.action = Marker.DELETEALL
        self.goal_map_marker_pub.publish(clear_map)

        clear_arm = Marker()
        clear_arm.header.frame_id = self.arm_base_frame
        clear_arm.header.stamp = now
        clear_arm.action = Marker.DELETEALL
        self.goal_arm_marker_pub.publish(clear_arm)

    def publish_pose_marker(self, pub, frame_id: str, pose, marker_id: int, ns: str, scale: float = 0.20):
        # Sphere marker
        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose = pose

        m.scale.x = scale * 0.25
        m.scale.y = scale * 0.25
        m.scale.z = scale * 0.25

        m.color.a = 1.0
        m.color.r = 1.0
        m.color.g = 0.2
        m.color.b = 0.2

        m.lifetime = DurationMsg(sec=0, nanosec=0)
        pub.publish(m)

        # Axes marker (use marker_id+1000 to avoid clashing with sphere ids)
        m_axes = self._make_axes_marker(
            frame_id=frame_id,
            pose=pose,
            marker_id=int(marker_id) + 1000,
            ns=f"{ns}_axes",
            axis_len=scale * 0.35,
            axis_width=scale * 0.03,
        )
        pub.publish(m_axes)
        
    def _make_color(self, r: float, g: float, b: float, a: float = 1.0) -> ColorRGBA:
        c = ColorRGBA()
        c.r = float(r); c.g = float(g); c.b = float(b); c.a = float(a)
        return c

    def _quat_multiply(self, q1, q2) -> tuple:
        """Hamilton product of two quaternions given as (x, y, z, w) tuples."""
        x1, y1, z1, w1 = float(q1[0]), float(q1[1]), float(q1[2]), float(q1[3])
        x2, y2, z2, w2 = float(q2[0]), float(q2[1]), float(q2[2]), float(q2[3])
        return (
            w1*x2 + x1*w2 + y1*z2 - z1*y2,
            w1*y2 - x1*z2 + y1*w2 + z1*x2,
            w1*z2 + x1*y2 - y1*x2 + z1*w2,
            w1*w2 - x1*x2 - y1*y2 - z1*z2,
        )

    def _quat_rotate_vec(self, q, v):
        """
        Rotate vector v by quaternion q (geometry_msgs Quaternion) without external deps.
        v: [x,y,z]
        returns rotated [x,y,z]
        """
        # q assumed normalized-ish
        qx, qy, qz, qw = float(q.x), float(q.y), float(q.z), float(q.w)
        vx, vy, vz = float(v[0]), float(v[1]), float(v[2])

        # t = 2 * cross(q.xyz, v)
        tx = 2.0 * (qy * vz - qz * vy)
        ty = 2.0 * (qz * vx - qx * vz)
        tz = 2.0 * (qx * vy - qy * vx)

        # v' = v + qw * t + cross(q.xyz, t)
        cx = (qy * tz - qz * ty)
        cy = (qz * tx - qx * tz)
        cz = (qx * ty - qy * tx)

        return [vx + qw * tx + cx, vy + qw * ty + cy, vz + qw * tz + cz]

    def _make_axes_marker(self, frame_id: str, pose, marker_id: int, ns: str,
                        axis_len: float = 0.12, axis_width: float = 0.01) -> Marker:
        """
        Draw a coordinate triad at pose.position, oriented by pose.orientation.
        X=red, Y=green, Z=blue.
        """
        x = float(pose.position.x)
        y = float(pose.position.y)
        z = float(pose.position.z)
        q = pose.orientation

        ex = self._quat_rotate_vec(q, [axis_len, 0.0, 0.0])
        ey = self._quat_rotate_vec(q, [0.0, axis_len, 0.0])
        ez = self._quat_rotate_vec(q, [0.0, 0.0, axis_len])

        origin = Point(x=x, y=y, z=z)
        px = Point(x=x + ex[0], y=y + ex[1], z=z + ex[2])
        py = Point(x=x + ey[0], y=y + ey[1], z=z + ey[2])
        pz = Point(x=x + ez[0], y=y + ez[1], z=z + ez[2])

        m = Marker()
        m.header.frame_id = frame_id
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = ns
        m.id = marker_id
        m.type = Marker.LINE_LIST
        m.action = Marker.ADD

        m.scale.x = float(axis_width)  # line width

        # segments: (origin->px), (origin->py), (origin->pz)
        m.points = [origin, px, origin, py, origin, pz]

        # per-vertex colors (must match points length)
        m.colors = [
            self._make_color(1.0, 0.0, 0.0, 1.0), self._make_color(1.0, 0.0, 0.0, 1.0),  # X red
            self._make_color(0.0, 1.0, 0.0, 1.0), self._make_color(0.0, 1.0, 0.0, 1.0),  # Y green
            self._make_color(0.0, 0.0, 1.0, 1.0), self._make_color(0.0, 0.0, 1.0, 1.0),  # Z blue
        ]

        # identity pose (we already placed points in frame coordinates)
        m.pose.orientation.w = 1.0

        m.lifetime = DurationMsg(sec=0, nanosec=0)
        return m

    # -----------------------------
    # Main logic
    # -----------------------------
    def arm_is_at_named_pose(
        self,
        target_pose: tuple[float, float, float, float, float, float, float],
        require_stable: bool = True,
    ) -> bool:
        current_pose = self._current_arm_tool_pose_in_arm_base()
        if current_pose is None:
            if require_stable:
                self._arm_named_pose_stable_since = None
            return False

        dx = float(current_pose.pose.position.x) - float(target_pose[0])
        dy = float(current_pose.pose.position.y) - float(target_pose[1])
        dz = float(current_pose.pose.position.z) - float(target_pose[2])
        pos_err = math.sqrt(dx * dx + dy * dy + dz * dz)

        current_q = (
            float(current_pose.pose.orientation.x),
            float(current_pose.pose.orientation.y),
            float(current_pose.pose.orientation.z),
            float(current_pose.pose.orientation.w),
        )
        target_q = (
            float(target_pose[3]),
            float(target_pose[4]),
            float(target_pose[5]),
            float(target_pose[6]),
        )
        ang_err = self._quat_angle_error_rad(current_q, target_q)

        if pos_err > self.arm_named_pose_pos_tol_m or ang_err > self.arm_named_pose_ang_tol_rad:
            if require_stable:
                self._arm_named_pose_stable_since = None
            return False

        if not require_stable:
            return True

        now = self.get_clock().now()
        if self._arm_named_pose_stable_since is None:
            self._arm_named_pose_stable_since = now
            return False

        return (now - self._arm_named_pose_stable_since) >= Duration(
            seconds=self.arm_named_pose_settle_time_s
        )

    def command_arm_named_position(
        self,
        position_name: str,
        expected_pose: tuple[float, float, float, float, float, float, float],
        stage: str,
        timeout_s: float,
    ):
        self._arm_named_pose_stable_since = None
        self._arm_execution_started = False
        self._arm_execution_completed = False

        client, srv_name = self._get_arm_position_client()
        if client is None or srv_name is None:
            self.get_logger().error(
                f"Arm position service not available. Tried: {self.arm_send_position_service_names}"
            )
            self.state = State.IDLE
            self._clear_plan()
            return

        req = SendPosition.Request()
        req.position_name = str(position_name)

        self.state = State.WAITING_ARM_POSITION
        self.arm_named_pose_deadline = self.get_clock().now() + Duration(seconds=timeout_s)
        self._pending_arm_position_name = str(position_name)
        self._pending_arm_position_pose = tuple(expected_pose)
        self._pending_arm_position_stage = str(stage)

        def _on_response(fut):
            try:
                resp = fut.result()
            except Exception as e:
                self.get_logger().error(f"Arm position service call failed for '{position_name}': {e}")
                self.state = State.IDLE
                self.arm_named_pose_deadline = None
                self._pending_arm_position_name = None
                self._pending_arm_position_pose = None
                self._pending_arm_position_stage = None
                self._arm_execution_started = False
                self._arm_execution_completed = False
                self._clear_plan()
                return

            if resp is None or not resp.success:
                message = resp.message if resp is not None else "empty response"
                self.get_logger().error(
                    f"Arm position '{position_name}' rejected by {srv_name}: {message}"
                )
                self.state = State.IDLE
                self.arm_named_pose_deadline = None
                self._pending_arm_position_name = None
                self._pending_arm_position_pose = None
                self._pending_arm_position_stage = None
                self._arm_execution_started = False
                self._arm_execution_completed = False
                self._clear_plan()
                return

            self.get_logger().info(
                f"Arm position '{position_name}' accepted by {srv_name}: {resp.message}"
            )

        client.call_async(req).add_done_callback(_on_response)
        self.get_logger().info(f"Requested arm position '{position_name}' via {srv_name}.")


    def is_planar_reachable(self, goal_map: PoseStamped) -> tuple[bool, float]:
        d = self.planar_distance_armbase_to_goal(goal_map)
        return (d <= self.reach_radius), d

    def is_vertical_reachable_given_goal_arm(self, goal_arm: PoseStamped) -> bool:
        z = float(goal_arm.pose.position.z)

        # If target is below the arm base, column won't help; don't fail routing on this criterion.
        if z < 0.0:
            return True

        return (self.arm_reachable_z_min <= z <= self.arm_reachable_z_max)

    def choose_column_height_for_goal_z(
        self,
        goal_z_in_arm_base: float,
        current_h: float | None = None,
    ) -> tuple[float | None, float, float]:
        """
        Compute the minimum absolute column height needed for a goal already expressed
        in the arm_base frame.

        Returns:
        (selected_height_or_None, adjusted_goal_z, delta_h)
        """
        if current_h is None:
            current_h = self.current_column_height_from_param()

        z0 = float(goal_z_in_arm_base)

        if z0 < 0.0:
            target_h = self.column_min_height_m
        else:
            target_h = max(
                self.column_min_height_m,
                current_h + z0 - self.arm_reachable_z_max,
            )
            target_h = min(target_h, self.column_max_height_m)

        delta_h = target_h - current_h
        z_adj = z0 - delta_h

        if not (self.arm_reachable_z_min <= z_adj <= self.arm_reachable_z_max or z0 < 0.0):
            return None, z0, 0.0

        return target_h, z_adj, delta_h

    # def apply_column_delta_to_goal_arm(self, goal_arm: PoseStamped, delta_h: float) -> PoseStamped:
    #     # delta_h > 0 means column raises arm_base -> goal in arm_base appears lower in z
    #     out = PoseStamped()
    #     out.header = goal_arm.header
    #     out.pose = goal_arm.pose
    #     out.pose.position.z = float(goal_arm.pose.position.z) - float(delta_h)
    #     return out
    
    def choose_column_height_for_goal(
        self,
        goal_map: PoseStamped,
    ) -> tuple[float | None, PoseStamped, float]:
        """
        Returns:
        (selected_height_or_None, adjusted_goal_arm, delta_h)

        adjusted_goal_arm is in arm_base frame with z adjusted as if column moved.
        delta_h = selected_height - current_height.
        """
        current_h = self.current_column_height_from_param()

        # Goal in arm_base at current column height (or best available TF)
        goal_arm = self.transform_goal_to_arm_base(goal_map)
        target_h, z_adj, delta_h = self.choose_column_height_for_goal_z(
            goal_z_in_arm_base=float(goal_arm.pose.position.z),
            current_h=current_h,
        )

        if target_h is None:
            return None, goal_arm, 0.0

        adjusted = PoseStamped()
        adjusted.header = goal_arm.header
        adjusted.pose = goal_arm.pose
        adjusted.pose.position.z = float(z_adj)
        return target_h, adjusted, delta_h
    
    def command_column(self, height_m: float):
        self._column_stable_since = None
        target_h = float(height_m)
        if self.column_control_mode == "trajectory_action":
            self._command_column_via_action(target_h)
            return

        self._command_column_via_topic(target_h)

    def _command_column_via_action(self, target_h: float):
        if self.column_client is None:
            self.get_logger().error("Column action client is not initialized.")
            self._reset_column_command()
            self._clear_plan()
            return

        if not self.column_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"Column action server not available: {self.column_action_name}")
            self._reset_column_command()
            self._clear_plan()
            return

        # Build FollowJointTrajectory goal (same structure as your CLI)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = [self.column_joint_name]

        pt = JointTrajectoryPoint()
        pt.positions = [target_h]

        move_time_s = float(self.column_move_time_s)
        sec = int(move_time_s)
        nanosec = int((move_time_s - sec) * 1e9)
        pt.time_from_start = DurationMsg(sec=sec, nanosec=nanosec)

        goal.trajectory.points = [pt]

        self._begin_column_wait(target_h)

        def _on_result(fut):
            try:
                res = fut.result().result
                # res.error_code, res.error_string are available in FollowJointTrajectory.Result
                if res.error_code != 0:
                    self.get_logger().warn(
                        f"Column trajectory finished with error_code={res.error_code} error_string='{res.error_string}'"
                    )
                else:
                    # Treat successful action completion as settled command intent.
                    # This prevents tiny encoder offsets from causing endless re-command loops.
                    snap_tol = max(self.column_tolerance_m, self.column_retract_tolerance_m)
                    if self.column_current_height is None:
                        self.column_current_height = target_h
                    elif abs(self.column_current_height - target_h) <= snap_tol:
                        self.column_current_height = target_h
                    self.column_current_velocity = 0.0
                    self.get_logger().info("Column trajectory action reported success.")
            except Exception as e:
                self.get_logger().warn(f"Column trajectory result callback failed: {e}")

        def _on_goal_response(fut):
            try:
                goal_handle = fut.result()
            except Exception as e:
                self.get_logger().error(f"Column send_goal failed: {e}")
                self._reset_column_command()
                self._clear_plan()
                return

            if not goal_handle.accepted:
                self.get_logger().error("Column trajectory goal was rejected by the controller.")
                self._reset_column_command()
                self._clear_plan()
                return

            self.get_logger().info("Column trajectory goal accepted.")
            goal_handle.get_result_async().add_done_callback(_on_result)

        try:
            send_future = self.column_client.send_goal_async(goal)
            send_future.add_done_callback(_on_goal_response)
        except Exception as e:
            self.get_logger().error(f"Exception while sending column trajectory goal: {e}")
            self._reset_column_command()
            self._clear_plan()
            return

        self.get_logger().info(
            f"Column action goal sent: target_height={target_h:.3f} m "
            f"joint='{self.column_joint_name}' time_from_start={move_time_s:.2f}s "
            f"on {self.column_action_name}"
        )

    def _command_column_via_topic(self, target_h: float):
        if self.column_command_pub is None:
            self.get_logger().error("Column command publisher is not initialized.")
            self._reset_column_command()
            self._clear_plan()
            return

        cmd = Float64MultiArray()
        cmd.data = [target_h]

        try:
            self.column_command_pub.publish(cmd)
        except Exception as e:
            self.get_logger().error(f"Failed to publish column command: {e}")
            self._reset_column_command()
            self._clear_plan()
            return

        self._begin_column_wait(target_h)
        self.get_logger().info(
            f"Column position command sent: target_height={target_h:.3f} m "
            f"on {self.column_command_topic}"
        )

    def current_column_height_from_param(self) -> float:
        if self.column_current_height is not None:
            return float(self.column_current_height)
        # fallback (if joint_states not received yet)
        return float(self.get_parameter("column_current_height").value)

    def column_reached_target(self) -> bool:
        if self.column_target_height is None:
            return True
        current_h = self.current_column_height_from_param()
        if abs(current_h - self.column_target_height) > self.column_tolerance_m:
            self._column_stable_since = None
            return False

        # If we have velocity, require it to be near zero for a short window
        if self.column_current_velocity is not None:
            if abs(self.column_current_velocity) > self.column_vel_tol:
                self._column_stable_since = None
                return False

            now = self.get_clock().now()
            if self._column_stable_since is None:
                self._column_stable_since = now
                return False

            if (now - self._column_stable_since) < Duration(seconds=self.column_settle_time_s):
                return False

        # Also require TF to be fresh (so transforms reflect the new joint state)
        return self._tf_pose_is_fresh()

    def on_goal(self, msg: PoseStamped):
        goal_map = self._normalize_goal_to_map(msg)
        if goal_map is None:
            return

        # New accepted goal -> clear stale visualizations from previous goal.
        self.clear_goal_markers()

        # Marker 1: received goal in map frame
        self.publish_pose_marker(
            pub=self.goal_map_marker_pub,
            frame_id=self.global_frame,
            pose=goal_map.pose,
            marker_id=self._marker_id_map,
            ns=f"{self._marker_ns}_map",
            scale=0.35
        )

        # Override current goal if needed
        if self.state in (
            State.WAITING_NAV2,
            State.WAITING_NAV2_SETTLE,
            State.WAITING_BASE_SERVICE,
            State.WAITING_COLUMN,
            State.WAITING_ARM_POSITION,
        ):
            self.get_logger().warn("New goal received while busy. Canceling Nav2 and overriding plan.")
            self.cancel_nav2_goal()
            self.state = State.IDLE

        self.active_goal_map = goal_map
        self.planned_column_height = None
        self.arm_fold_done_for_goal = False
        self.arm_unfold_done_for_goal = False
        self.base_move_done_for_goal = False

        self.plan_and_execute()
        
    def cancel_nav2_goal(self):
        if self.nav2_goal_handle is None:
            self._clear_plan()
            self.nav2_goal_in_flight = False
            self.state = State.IDLE
            return
        try:
            cancel_future = self.nav2_goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda f: self.get_logger().info("Nav2 goal cancel requested."))
        except Exception as e:
            self.get_logger().warn(f"Failed to cancel Nav2 goal: {e}")

        self._clear_plan()
        self.nav2_goal_handle = None
        self.nav2_goal_in_flight = False
        self.state = State.IDLE

    def _normalize_goal_to_map(self, msg: PoseStamped) -> PoseStamped | None:
        if not msg.header.frame_id:
            self.get_logger().warn("Received goal with empty frame_id. Ignoring.")
            return None

        if msg.header.frame_id == self.global_frame:
            return msg

        self.get_logger().warn(
            f"Goal frame_id='{msg.header.frame_id}' expected '{self.global_frame}'. Trying TF -> map."
        )
        try:
            return self.transform_pose_stamped(msg, self.global_frame)
        except Exception as e:
            self.get_logger().error(f"Cannot transform goal to {self.global_frame}: {e}")
            return None

    def plan_and_execute(self):
        """
        Routing logic:

        - If planar reachable:
            - if vertical reachable -> stage arm as 'unfolded' and publish arm goal
            - else -> command column (ONLY in this planar case)
        - If NOT planar reachable (base must move):
            - fold arm first via position_sender_node (safety)
            - retract column to 0.0 (safety)
            - call base service -> Nav2
            - after Nav2 settle -> re-run plan_and_execute()
            (then column extension and unfold happen AFTER base)
        """
        if self.active_goal_map is None:
            return
        if self.state != State.IDLE:
            return

        goal_map = self.active_goal_map

        # First decide planar reachability (no need for goal->arm_base yet)
        planar_ok, d = self.is_planar_reachable(goal_map)

        # Prevent Nav2 re-dispatch loops near the planar boundary after a successful base move.
        if (not planar_ok) and self.base_move_done_for_goal:
            relaxed_limit = self.reach_radius + self.reach_radius_post_nav2_margin_m
            if d <= relaxed_limit:
                self.get_logger().info(
                    f"Post-Nav2 planar distance d={d:.3f} exceeds strict reach_radius={self.reach_radius:.3f} "
                    f"but is within relaxed_limit={relaxed_limit:.3f}. Continuing without another base move."
                )
                planar_ok = True
            else:
                self.get_logger().error(
                    f"Post-Nav2 planar distance still too large (d={d:.3f} > relaxed_limit={relaxed_limit:.3f}). "
                    f"Goal unreachable. Aborting to prevent Nav2 loop."
                )
                self._clear_plan()
                return

        # -------------------------
        # CASE A: Planar reachable (no base move)
        # -------------------------
        if planar_ok:
            # Now we need the goal in arm_base to check vertical reach
            try:
                goal_arm_now = self.transform_goal_to_arm_base(goal_map)
            except Exception as e:
                self.get_logger().warn(f"TF not ready: {e}")
                return

            # Prefer retracting the column if the goal is below the arm_base (z < 0)
            current_h = self.current_column_height_from_param()
            if (
                float(goal_arm_now.pose.position.z) < 0.0
                and abs(current_h - self.column_min_height_m) > self.column_retract_tolerance_m
            ):
                self.get_logger().info(
                    f"Goal below arm_base (z={goal_arm_now.pose.position.z:.3f}) while column extended "
                    f"(h={current_h:.3f}). Retracting column to {self.column_min_height_m:.3f} before arm motion."
                )
                self.command_column(self.column_min_height_m)
                return
            
            # Always choose the minimum column height that makes the goal reachable
            h, goal_arm_adj, delta_h = self.choose_column_height_for_goal(goal_map)
            if h is None:
                self.get_logger().warn(
                    f"Planar reachable (d={d:.3f}) but no admissible column height makes goal vertically reachable."
                )
                self._clear_plan()
                return

            current_h = self.current_column_height_from_param()
            if abs(h - current_h) <= self.column_tolerance_m:
                # already there, publish with current TF
                try:
                    goal_arm_now = self.transform_goal_to_arm_base(goal_map)
                except Exception as e:
                    self.get_logger().warn(f"TF not ready: {e}")
                    return

                if self.arm_unfold_enable and (not self.arm_unfold_done_for_goal):
                    if self.arm_is_at_named_pose(self.arm_unfold_pose, require_stable=False):
                        self.arm_unfold_done_for_goal = True
                    else:
                        self.get_logger().info(
                            "Arm goal ready: staging arm through position_sender_node 'unfolded' first."
                        )
                        self.command_arm_named_position(
                            position_name=self.arm_unfold_position_name,
                            expected_pose=self.arm_unfold_pose,
                            stage="unfold",
                            timeout_s=self.arm_unfold_timeout_s,
                        )
                        return

                self.publish_arm_goal(goal_arm_now, note="planar ok; column already at needed height")
                self._clear_plan()
                return

            self.command_column(h)
            return

        # -------------------------
        # CASE B: NOT planar reachable -> base must move
        # -------------------------

        # 1) Fold arm ONLY when base must move (before any column motion)
        if self.arm_fold_enable and (not self.arm_fold_done_for_goal):
            if self.arm_is_at_named_pose(self.arm_fold_pose, require_stable=False):
                self.arm_fold_done_for_goal = True
                self.arm_unfold_done_for_goal = False
            else:
                self.get_logger().info(
                    "Base move needed: staging arm through position_sender_node 'folded' first."
                )
                self.command_arm_named_position(
                    position_name=self.arm_fold_position_name,
                    expected_pose=self.arm_fold_pose,
                    stage="fold",
                    timeout_s=self.arm_fold_timeout_s,
                )
                return

        # 2) Retract column to the minimum height before base motion
        current_h = self.current_column_height_from_param()
        if abs(current_h - self.column_min_height_m) > self.column_retract_tolerance_m:
            self.get_logger().info(
                f"Base move needed: retracting column to {self.column_min_height_m:.3f} before Nav2 "
                f"(current={current_h:.3f} m)."
            )
            self.command_column(self.column_min_height_m)
            return

        # 3) Column is retracted -> compute corrected arm z from TF at column=0 and call base service/Nav2
        try:
            goal_arm_now = self.transform_goal_to_arm_base(goal_map)
        except Exception as e:
            self.get_logger().warn(f"TF not ready after fold/retract pre-Nav2: {e}")
            return

        planned_column_height, corrected_arm_z, _delta_h = self.choose_column_height_for_goal_z(
            goal_z_in_arm_base=float(goal_arm_now.pose.position.z),
            current_h=current_h,
        )
        if planned_column_height is None:
            self.get_logger().warn(
                f"Base move needed but no admissible column height makes goal vertically reachable "
                f"after base placement (goal_z_in_arm_base={goal_arm_now.pose.position.z:.3f})."
            )
            self._clear_plan()
            return

        self.planned_column_height = planned_column_height
        self.get_logger().info(
            f"Precomputing post-Nav2 column plan: current_h={current_h:.3f} m, "
            f"target_h={planned_column_height:.3f} m, corrected_arm_z={corrected_arm_z:.3f} m"
        )

        self.call_compute_base_service_for_goal_map_with_corrected_z(
            corrected_arm_z=corrected_arm_z,
            d=d
        )
        return
        
    def _clear_plan(self):
        self.active_goal_map = None
        self.planned_column_height = None
        self.base_move_done_for_goal = False
        self.arm_named_pose_deadline = None
        self._arm_named_pose_stable_since = None
        self._pending_arm_position_name = None
        self._pending_arm_position_pose = None
        self._pending_arm_position_stage = None
        self._arm_execution_started = False
        self._arm_execution_completed = False
        # self.planned_column_delta_h = None
        
    def publish_arm_goal(self, goal_arm: PoseStamped, note: str = ""):
        # Marker 2: goal in arm_base frame
        self.publish_pose_marker(
            pub=self.goal_arm_marker_pub,
            frame_id=self.arm_base_frame,
            pose=goal_arm.pose,
            marker_id=self._marker_id_arm,
            ns=f"{self._marker_ns}_arm",
            scale=0.25
        )

        self.arm_goal_pub.publish(goal_arm)
        if note:
            self.get_logger().info(f"Arm goal published: {note}")

    def call_compute_base_service_for_goal_map_with_corrected_z(self, corrected_arm_z: float, d: float):
        if self.active_goal_map is None:
            self.get_logger().error("Internal error: active_goal_map is None while calling base service.")
            self.state = State.IDLE
            return

        which = self._select_base_service()
        if which is None:
            self.get_logger().error(
                f"No base service available. Tried '{self.compute_optimal_base_srv}' then '{self.compute_base_placement_srv}'."
            )
            self._clear_plan()
            return
        
        if self.active_goal_map is not None:
            g = self.active_goal_map.pose.position
            self.get_logger().info(f"Active map goal: ({g.x:.3f}, {g.y:.3f}, {g.z:.3f}), planned_column_height={self.planned_column_height}")

        self.state = State.WAITING_BASE_SERVICE

        # Preferred: OptimalBase
        if which == "optimal":
            if not self.optimal_srv.wait_for_service(timeout_sec=2.5):
                self.get_logger().warn("compute_optimal_base detected but not reachable now. Falling back to placement.")
                which = "placement"
            else:
                req = OptimalBase.Request()

                gx = float(self.active_goal_map.pose.position.x)
                gy = float(self.active_goal_map.pose.position.y)
                z = float(corrected_arm_z)

                # orientation: use map goal orientation (or keep identity if you prefer)
                q = self.active_goal_map.pose.orientation
                r, p, yaw = self._quat_to_rpy_deg(q)

                x = gx
                y = gy

                req.poses_ee_xyzrpy = [x, y, z, r, p, yaw]
                req.obstacle_rects = []
                req.obstacle_circles = []
                req.min_dist = float(self.get_parameter("optimal_min_dist").value)
                req.round_decimals = int(self.get_parameter("optimal_round_decimals").value)
                req.grid_res = float(self.get_parameter("optimal_grid_res").value)

                # Center limits around the *map* goal (because base search is in map space)
                gx = float(self.active_goal_map.pose.position.x)
                gy = float(self.active_goal_map.pose.position.y)
                margin = float(self.get_parameter("optimal_limits_margin").value)
                req.x_limits = [gx - margin, gx + margin]
                req.y_limits = [gy - margin, gy + margin]

                req.enable_simulator = bool(self.get_parameter("optimal_enable_simulator").value)
                req.enable_robot_viz = bool(self.get_parameter("optimal_enable_robot_viz").value)

                self.get_logger().info(f"Calling compute_optimal_base (d={d:.3f}) with EE goal in arm_base.")
                self.get_logger().info(
                    "OptimalBase request:\n"
                    f"  poses_ee_xyzrpy={req.poses_ee_xyzrpy}\n"
                    f"  min_dist={req.min_dist}, round_decimals={req.round_decimals}, grid_res={req.grid_res}\n"
                    f"  x_limits={req.x_limits}, y_limits={req.y_limits}\n"
                    f"  obstacle_rects={len(req.obstacle_rects)}, obstacle_circles={len(req.obstacle_circles)}\n"
                    f"  enable_simulator={req.enable_simulator}, enable_robot_viz={req.enable_robot_viz}"
                )
                future = self.optimal_srv.call_async(req)
                future.add_done_callback(self.on_optimal_base_response)
                return

        # Fallback: ComputeBasePlacement
        if not self.base_srv.wait_for_service(timeout_sec=2.5):
            self.get_logger().error("Service /compute_base_placement not available.")
            self.state = State.IDLE
            self._clear_plan()
            return

        req = ComputeBasePlacement.Request()
        pmsg = Pose()
        pmsg.position.x = float(self.active_goal_map.pose.position.x)
        pmsg.position.y = float(self.active_goal_map.pose.position.y)
        pmsg.position.z = float(corrected_arm_z)
        pmsg.orientation = self.active_goal_map.pose.orientation
        req.targets = [pmsg]

        self.get_logger().info(f"Calling compute_base_placement (d={d:.3f}) with EE goal in arm_base.")
        self.get_logger().info(
            "ComputeBasePlacement request:\n"
            f"  targets_count={len(req.targets)}\n"
            f"  target[0].position=({req.targets[0].position.x:.3f}, {req.targets[0].position.y:.3f}, {req.targets[0].position.z:.3f})\n"
            f"  target[0].orientation=({req.targets[0].orientation.x:.3f}, {req.targets[0].orientation.y:.3f}, "
            f"{req.targets[0].orientation.z:.3f}, {req.targets[0].orientation.w:.3f})"
        )
        future = self.base_srv.call_async(req)
        future.add_done_callback(self.on_base_placement_response)

    def on_optimal_base_response(self, future):
        self.state = State.IDLE
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"compute_optimal_base call failed: {e}")
            self._clear_plan()
            return
        if not resp.success:
            self.get_logger().error(f"compute_optimal_base success=false: {resp.message}")
            self._clear_plan()
            return

        # The service returns the desired arm_base pose in map frame.
        # Nav2 steers base_link, so we must compose:
        #   T_map_base_link = T_map_arm_base_desired * T_arm_base_base_link
        arm_base_x = float(resp.base_x)
        arm_base_y = float(resp.base_y)
        q_desired = (float(resp.base_qx), float(resp.base_qy), float(resp.base_qz), float(resp.base_qw))

        try:
            tf_ab_bl = self.tf_buffer.lookup_transform(
                self.arm_base_frame, self.robot_base_frame, rclpy.time.Time(), self._tf_timeout()
            )
        except Exception as e:
            self.get_logger().error(
                f"Cannot lookup TF {self.arm_base_frame} -> {self.robot_base_frame}: {e}"
            )
            self._clear_plan()
            return

        # Offset of base_link origin in arm_base frame
        off = tf_ab_bl.transform.translation
        off_q = tf_ab_bl.transform.rotation

        # Rotate the offset by the desired arm_base orientation in map
        class _Q:
            pass
        _q = _Q()
        _q.x, _q.y, _q.z, _q.w = q_desired
        rotated_off = self._quat_rotate_vec(_q, [off.x, off.y, off.z])
        bl_x = arm_base_x + rotated_off[0]
        bl_y = arm_base_y + rotated_off[1]

        # Compose orientations: Q_base_link_in_map = Q_desired * Q_arm_base_to_base_link
        bl_q = self._quat_multiply(q_desired, (off_q.x, off_q.y, off_q.z, off_q.w))

        self.get_logger().info(
            f"OptimalBase arm_base target: ({arm_base_x:.3f}, {arm_base_y:.3f}) -> "
            f"base_link target: ({bl_x:.3f}, {bl_y:.3f})"
        )

        base_goal_map = PoseStamped()
        base_goal_map.header.frame_id = self.global_frame
        base_goal_map.header.stamp = self.get_clock().now().to_msg()
        base_goal_map.pose.position.x = bl_x
        base_goal_map.pose.position.y = bl_y
        base_goal_map.pose.position.z = 0.0
        base_goal_map.pose.orientation.x = float(bl_q[0])
        base_goal_map.pose.orientation.y = float(bl_q[1])
        base_goal_map.pose.orientation.z = float(bl_q[2])
        base_goal_map.pose.orientation.w = float(bl_q[3])

        self.send_nav2_goal(base_goal_map)
        
    def on_base_placement_response(self, future):
        self.state = State.IDLE
        try:
            resp = future.result()
        except Exception as e:
            self.get_logger().error(f"compute_base_placement call failed: {e}")
            self._clear_plan()
            return
        if not resp.success:
            self.get_logger().error(f"compute_base_placement success=false: {resp.message}")
            self._clear_plan()
            return

        base_goal_map = PoseStamped()
        base_goal_map.header.frame_id = self.global_frame
        base_goal_map.header.stamp = self.get_clock().now().to_msg()
        base_goal_map.pose = resp.best_base

        self.send_nav2_goal(base_goal_map)

    def send_nav2_goal(self, base_goal_map: PoseStamped):
        if self.nav2_goal_in_flight:
            self.get_logger().warn("Nav2 goal already in flight. Cancelling previous goal.")
            return

        if not self.nav2_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error(f"Nav2 action server not available: {self.nav2_action_name}")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = base_goal_map

        self.get_logger().info(
            f"Sending Nav2 goal: frame={goal_msg.pose.header.frame_id} "
            f"x={goal_msg.pose.pose.position.x:.3f}, y={goal_msg.pose.pose.position.y:.3f}"
        )

        self.state = State.WAITING_NAV2
        self.nav2_goal_in_flight = True
        self.base_move_done_for_goal = False

        send_future = self.nav2_client.send_goal_async(goal_msg, feedback_callback=self.on_nav2_feedback)
        send_future.add_done_callback(self.on_nav2_goal_response)

    def on_nav2_feedback(self, feedback_msg):
        # Optional: feedback_msg.feedback.current_pose, distance_remaining, etc.
        pass

    def on_nav2_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 send_goal failed: {e}")
            self.nav2_goal_in_flight = False
            self.state = State.IDLE
            self._clear_plan()
            return

        if not goal_handle.accepted:
            self.get_logger().error("Nav2 goal rejected.")
            self.nav2_goal_in_flight = False
            self.state = State.IDLE
            self._clear_plan()
            return

        self.nav2_goal_handle = goal_handle
        self.get_logger().info("Nav2 goal accepted. Waiting for result...")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.on_nav2_result)

    def on_nav2_result(self, future):
        self.nav2_goal_in_flight = False
        self.nav2_goal_handle = None

        try:
            result = future.result()
        except Exception as e:
            self.get_logger().error(f"Nav2 result failed: {e}")
            self.state = State.IDLE
            self._clear_plan()
            return

        if result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(f"Nav2 did not succeed. status={result.status}")
            self.state = State.IDLE
            self._clear_plan()
            return

        if self.active_goal_map is None:
            self.get_logger().warn("Nav2 succeeded but no active goal is stored.")
            self.state = State.IDLE
            return

        # Start settling phase (base stopped + TF fresh&stable)
        self.state = State.WAITING_NAV2_SETTLE
        now = self.get_clock().now()
        self._base_stable_since = None

        # Ensure the deadline can actually accommodate the required stable windows
        min_timeout = self.base_settle_time_s + self.tf_stable_required_s + self.tf_fresh_max_age_s
        timeout_s = max(self.tf_settle_after_nav2_s, min_timeout)
        self.nav2_settle_deadline = now + Duration(seconds=timeout_s)        
        self._nav2_tf_last = None
        self._nav2_tf_last_yaw = None
        self._nav2_tf_stable_since = None


def main(args=None):
    rclpy.init(args=args)
    node = GoalRouter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
