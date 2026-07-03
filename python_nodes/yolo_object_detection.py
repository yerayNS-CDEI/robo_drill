#!/usr/bin/env python3

from ament_index_python.packages import get_package_share_directory

# Ensure conda environment libraries are found
import sys
import os
conda_prefix = os.path.expanduser('~/miniconda3/envs/yolo_discover')
sys.path.insert(0, f'{conda_prefix}/lib/python3.10/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, PoseArray, Pose
from sensor_msgs.msg import Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import tf2_ros
from tf2_ros import Buffer, TransformListener
import tf2_geometry_msgs  # noqa: F401  (registers PointStamped transform)
import csv
from datetime import datetime
from ultralytics import YOLO
import depthai as dai
import numpy as np
import cv2
import torch
import time
import logging
from threading import Lock, Thread


# --------------------------- geometry helpers ---------------------------
def deproject(u, v, z, fx, fy, cx, cy):
    """Pixel (u,v) + depth z (m) -> 3D point (m) in the camera OPTICAL frame."""
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z], dtype=np.float64)


def transform_to_Rt(tf):
    """geometry_msgs TransformStamped -> (3x3 rotation, 3 translation) as numpy."""
    q = tf.transform.rotation
    t = tf.transform.translation
    x, y, z, w = q.x, q.y, q.z, q.w
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ], dtype=np.float64)
    return R, np.array([t.x, t.y, t.z], dtype=np.float64)


class YOLODetectionNode(Node):
    """
    YOLO detection on the OAK-D Pro mounted on the robot with
    stereo depth. For each detected door/window it computes the two bounding-box
    corners and the surface plane in 3D, transforms them into the map frame via
    TF2, logs to CSV, and publishes markers + poses for the navigation stack.
    """

    def __init__(self):
        super().__init__('yolo_detection_node')

        # ---- parameters ----
        self.declare_parameter('model_path', 'best.pt')
        self.declare_parameter('preview_size', [1280, 720])
        self.declare_parameter('fps', 15)
        self.declare_parameter('csv_output_dir', 'rgb_detections')
        self.declare_parameter('odom_topic', '/rtabmap/odom')
        self.declare_parameter('end_effector_topic', '/end_effector_pose')
        self.declare_parameter('display_window', True)
        self.declare_parameter('conf_threshold', 0.6)
        # 3D / TF
        self.declare_parameter('target_classes', ['door', 'window'])
        self.declare_parameter('camera_optical_frame', 'oak_rgb_camera_optical_frame')
        # Legacy parameter kept only so older launch commands still parse.
        self.declare_parameter('ee_frame', 'tool0')
        self.declare_parameter('map_frame', 'map')
        # Intermediate frame for the two-hop map transform (see to_map).
        self.declare_parameter('odom_frame', 'odom')
        # Robot base frame; its pose in the map frame is logged as robot_pose.
        self.declare_parameter('robot_base_frame', 'turret_footprint')
        self.declare_parameter('max_range_m', 8.0)
        self.declare_parameter('min_plane_points', 50)
        # Lidar depth: prefer the lidar point cloud for box depth, fall back to
        # OAK stereo when too few cloud points project into the box.
        self.declare_parameter('use_lidar_depth', True)
        self.declare_parameter('cloud_topic', '/combined_cloud_filtered')
        self.declare_parameter('min_lidar_points', 20)
        # Central fraction of each box used to sample one robust median depth
        # (same idea as yolo_discover_3d.py --roi; avoids background that bleeds
        # in around the box edges).
        self.declare_parameter('depth_roi_frac', 0.5)
        self.declare_parameter('force_usb2', False)
        # Legacy hand-eye parameters kept for backward compatibility only.
        # The camera transform now comes from the robot's TF tree/URDF.
        self.declare_parameter('handeye_xyz', [0.0, 0.0, 0.0])
        self.declare_parameter('handeye_quat', [0.0, 0.0, 0.0, 1.0])
        self.declare_parameter('handeye_calibrated', False)

        gp = self.get_parameter
        self.model_path = gp('model_path').value
        self.preview_size = tuple(gp('preview_size').value)
        self.fps = gp('fps').value
        self.csv_output_dir = gp('csv_output_dir').value
        self.odom_topic = gp('odom_topic').value
        self.end_effector_topic = gp('end_effector_topic').value
        self.display_window = gp('display_window').value
        self.conf_threshold = gp('conf_threshold').value
        self.target_classes = {c.lower() for c in gp('target_classes').value}
        self.camera_optical_frame = gp('camera_optical_frame').value
        self.ee_frame = gp('ee_frame').value
        self.map_frame = gp('map_frame').value
        self.odom_frame = gp('odom_frame').value
        self.robot_base_frame = gp('robot_base_frame').value
        self.max_range_m = gp('max_range_m').value
        self.min_plane_points = gp('min_plane_points').value
        self.use_lidar_depth = gp('use_lidar_depth').value
        self.cloud_topic = gp('cloud_topic').value
        self.min_lidar_points = gp('min_lidar_points').value
        self.depth_roi_frac = gp('depth_roi_frac').value
        self.force_usb2 = gp('force_usb2').value
        self.handeye_xyz = [float(v) for v in gp('handeye_xyz').value]
        self.handeye_quat = [float(v) for v in gp('handeye_quat').value]
        self.handeye_calibrated = gp('handeye_calibrated').value

        # Robot/camera pose storage (kept for CSV; geometry uses TF2)
        self.robot_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
        self.pose_lock = Lock()
        self.camera_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0, 'qx': 0.0, 'qy': 0.0, 'qz': 0.0, 'qw': 1.0}
        self.camera_pose_lock = Lock()

        # Latest lidar cloud as (Nx3 float32 points in its own frame, frame_id).
        self.latest_cloud = None
        self.cloud_lock = Lock()
        self._cloud_logged = False

        # ---- output dir + CSV ----
        pkg_share = get_package_share_directory('robo_drill')
        req = os.path.expanduser(self.csv_output_dir)
        self.csv_output_dir = os.path.abspath(req if os.path.isabs(req) else os.path.join(pkg_share, req))
        os.makedirs(self.csv_output_dir, exist_ok=True)
        self.get_logger().info(f"Writing detections to: {self.csv_output_dir}")
        ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_filename = os.path.join(self.csv_output_dir, f'detections_{ts}.csv')
        self.init_csv_file()
        logging.basicConfig(filename=os.path.join(self.csv_output_dir, f'yolo_ros2_{ts}.log'),
                            level=logging.INFO, format='%(asctime)s: %(message)s')

        # ---- device / model ----
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.get_logger().info(f"Using device: {self.device}")
        try:
            self.model = YOLO(self.model_path)
            self.model.to(self.device)
            self.get_logger().info(f"Model loaded on {self.device}")
        except Exception as e:
            self.get_logger().error(f"Error loading model: {e}")
            raise

        # ---- TF2 ----
        # 60 s cache (default is 10 s): some transforms on the camera->odom path
        # update only in sporadic bursts (tens of seconds apart), so the chain's
        # latest commonly-valid time lags wall-clock; a longer cache keeps the
        # other links' samples around so the lookup resolves (slightly stale,
        # which is fine while scanning stationary) instead of extrapolating.
        self.tf_buffer = Buffer(cache_time=Duration(seconds=60.0))
        # spin_thread=True: ingest /tf on a dedicated thread. The 15 fps YOLO
        # inference monopolizes the node's single-threaded executor, which would
        # otherwise starve the /tf subscription and let the buffer go stale --
        # making lookups request a frozen timestamp that rtabmap's short buffer
        # has already aged past ("extrapolation into the past"). The Buffer is
        # thread-safe, so reading it from the timer callback is fine.
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)
        self._log_tf_configuration()

        # ---- publishers ----
        self.marker_pub = self.create_publisher(MarkerArray, '~/detected_openings_markers', 10)
        self.pose_pub = self.create_publisher(PoseArray, '~/detected_openings', 10)

        # ---- subscriptions (topic poses are fallbacks; TF in camera_callback wins) ----
        # These run on the main executor and are starved by the YOLO inference, so
        # robot_pose/camera_pose are instead populated from TF every frame (which
        # is reliably fresh via the spin_thread listener). The subscriptions remain
        # only as a fallback for when TF is unavailable.
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        self.create_subscription(PoseStamped, self.end_effector_topic, self.end_effector_callback, 10)

        # Lidar cloud: subscribed on a DEDICATED node/executor/thread (the same
        # pattern the TF listener uses) so the YOLO inference timer can't starve
        # it -- a stale cloud would deproject against a current TF and place
        # detections wrongly. SensorDataQoS matches the best-effort lidar publisher.
        if self.use_lidar_depth:
            self.cloud_node = rclpy.create_node('yolo_detection_cloud')
            self.cloud_node.create_subscription(
                PointCloud2, self.cloud_topic, self.cloud_callback, qos_profile_sensor_data)
            self.cloud_executor = SingleThreadedExecutor()
            self.cloud_executor.add_node(self.cloud_node)
            self.cloud_thread = Thread(target=self.cloud_executor.spin, daemon=True)
            self.cloud_thread.start()
            self.get_logger().info(f"Lidar depth enabled; subscribing to {self.cloud_topic}")

        # ---- camera ----
        self.depth_frame = None  # latest aligned depth (uint16 mm)
        self.init_camera()

        self.timer = self.create_timer(1.0 / self.fps, self.camera_callback)
        self.start_time = time.time()
        self.frame_count = 0
        self.current_fps = 0.0
        self.get_logger().info('YOLO 3D Detection Node initialized')

    # ------------------------------------------------------------------
    def _log_tf_configuration(self):
        legacy_handeye_active = (
            self.ee_frame != 'tool0'
            or any(abs(v) > 1e-9 for v in self.handeye_xyz)
            or any(abs(v) > 1e-9 for v in self.handeye_quat[:3])
            or abs(self.handeye_quat[3] - 1.0) > 1e-9
            or self.handeye_calibrated
        )
        if legacy_handeye_active:
            self.get_logger().warn(
                "Legacy ee_frame/handeye_* parameters are ignored. "
                f"Using TF published by the robot model instead; expecting "
                f"{self.camera_optical_frame} -> {self.map_frame} to already exist."
            )
        else:
            self.get_logger().info(
                f"Using TF tree for camera geometry; expecting "
                f"{self.camera_optical_frame} -> {self.map_frame}."
            )

    def init_csv_file(self):
        with open(self.csv_filename, 'w', newline='') as f:
            csv.writer(f).writerow([
                'timestamp', 'detection_id', 'class_name', 'confidence_percent',
                'bbox_x1', 'bbox_y1', 'bbox_x2', 'bbox_y2',
                # camera-frame box center (valid when depth is sufficient)
                'center_cam_x', 'center_cam_y', 'center_cam_z',
                # map-frame box center (valid when camera_optical_frame -> map TF exists)
                'map_valid',
                'center_map_x', 'center_map_y', 'center_map_z',
                # robot pose snapshot + camera pose snapshot
                'robot_pos_x', 'robot_pos_y', 'robot_pos_z',
                'robot_orient_x', 'robot_orient_y', 'robot_orient_z', 'robot_orient_w',
                'camera_pos_x', 'camera_pos_y', 'camera_pos_z',
                'camera_orient_x', 'camera_orient_y', 'camera_orient_z', 'camera_orient_w',
            ])

    def init_camera(self):
        """OAK-D Pro pipeline: RGB preview + stereo depth aligned to RGB."""
        W, H = self.preview_size
        pipeline = dai.Pipeline()
        cam = pipeline.create(dai.node.ColorCamera)
        cam.setBoardSocket(dai.CameraBoardSocket.CAM_A)
        cam.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        cam.setInterleaved(False)
        cam.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        cam.setPreviewSize(W, H)
        cam.setPreviewKeepAspectRatio(False)
        cam.setFps(self.fps)

        mono_l = pipeline.create(dai.node.MonoCamera)
        mono_r = pipeline.create(dai.node.MonoCamera)
        mono_l.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_r.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        mono_l.setCamera("left")
        mono_r.setCamera("right")
        mono_l.setFps(self.fps)
        mono_r.setFps(self.fps)

        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setLeftRightCheck(True)
        stereo.setSubpixel(True)
        stereo.setOutputSize(W, H)
        mono_l.out.link(stereo.left)
        mono_r.out.link(stereo.right)

        x_rgb = pipeline.create(dai.node.XLinkOut); x_rgb.setStreamName("rgb")
        cam.preview.link(x_rgb.input)
        x_depth = pipeline.create(dai.node.XLinkOut); x_depth.setStreamName("depth")
        stereo.depth.link(x_depth.input)

        try:
            if self.force_usb2:
                self.oak_device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
            else:
                try:
                    self.oak_device = dai.Device(pipeline)
                except RuntimeError as e:
                    self.get_logger().warn(f"Normal boot failed ({e}); retrying USB2/HIGH")
                    self.oak_device = dai.Device(pipeline, maxUsbSpeed=dai.UsbSpeed.HIGH)
            self.q_rgb = self.oak_device.getOutputQueue("rgb", maxSize=4, blocking=False)
            self.q_depth = self.oak_device.getOutputQueue("depth", maxSize=4, blocking=False)
            K = np.array(self.oak_device.readCalibration().getCameraIntrinsics(
                dai.CameraBoardSocket.CAM_A, W, H))
            self.fx, self.fy = K[0, 0], K[1, 1]
            self.cx, self.cy = K[0, 2], K[1, 2]
            self.get_logger().info(
                f"OAK ready ({self.oak_device.getUsbSpeed().name}); "
                f"intrinsics fx={self.fx:.1f} fy={self.fy:.1f} cx={self.cx:.1f} cy={self.cy:.1f}")
            if self.display_window:
                cv2.namedWindow("YOLO 3D Detections", cv2.WINDOW_NORMAL)
                cv2.resizeWindow("YOLO 3D Detections", W, H)
        except Exception as e:
            self.get_logger().error(f"Camera initialization error: {e}")
            raise

    # ------------------------------------------------------------------
    def odom_callback(self, msg):
        with self.pose_lock:
            p, o = msg.pose.pose.position, msg.pose.pose.orientation
            self.robot_pose.update(x=p.x, y=p.y, z=p.z, qx=o.x, qy=o.y, qz=o.z, qw=o.w)

    def end_effector_callback(self, msg):
        with self.camera_pose_lock:
            p, o = msg.pose.position, msg.pose.orientation
            self.camera_pose.update(x=p.x, y=p.y, z=p.z, qx=o.x, qy=o.y, qz=o.z, qw=o.w)

    def cloud_callback(self, msg):
        """Cache the latest lidar cloud as an Nx3 float32 array in its own frame.

        Fields are x,y,z(,intensity) as float32, so the first 3 floats of each
        point_step are the coordinates; parse with numpy directly (fast for the
        ~65k-point cloud) rather than the slow generator in sensor_msgs_py."""
        n = msg.width * msg.height
        if n == 0:
            return
        floats_per_pt = msg.point_step // 4
        pts = (np.frombuffer(msg.data, dtype=np.float32)
               .reshape(n, floats_per_pt)[:, :3].copy())
        pts = pts[np.isfinite(pts).all(axis=1)]
        with self.cloud_lock:
            self.latest_cloud = (pts, msg.header.frame_id)
        if not self._cloud_logged:
            self._cloud_logged = True
            self.get_logger().info(
                f"First lidar cloud: {n} pts in frame '{msg.header.frame_id}'")

    # ------------------------------------------------------------------
    def _near_surface_depth(self, valid_mm, min_points=None):
        """Depth (m) of the NEAREST substantial surface among the given valid
        depths (in mm), or None if too few points. `min_points` defaults to the
        camera threshold; lidar passes its own (sparser) threshold.

        A plain median is pulled toward the far scene seen *through* a glazed
        window; instead we histogram the depths and take the nearest bin that
        still holds a real surface's worth of points (so sparse near speckle is
        ignored too), then return the median of points around that bin."""
        min_points = self.min_plane_points if min_points is None else min_points
        z = valid_mm.astype(np.float64) / 1000.0
        z = z[(z > 0.2) & (z < self.max_range_m)]
        if z.size < min_points:
            return None
        bin_w = 0.10                                   # 10 cm depth bins
        edges = np.arange(z.min(), z.max() + bin_w, bin_w)
        if edges.size < 2:                             # all within one bin
            return float(np.median(z))
        counts, _ = np.histogram(z, bins=edges)
        # a real surface forms a peak; require a bin to hold a decent fraction of
        # the tallest peak so we skip flying-pixel speckle in front of the frame
        thresh = max(0.3 * counts.max(), min_points / 3.0)
        near_bin = int(np.argmax(counts >= thresh))    # first (nearest) such bin
        lo = edges[near_bin] - bin_w
        hi = edges[near_bin] + 2 * bin_w               # ~30 cm window on the surface
        sel = z[(z >= lo) & (z <= hi)]
        return float(np.median(sel)) if sel.size else float(np.median(z))

    def box_3d(self, depth, x1, y1, x2, y2):
        """3D coordinate of the box CENTER in the camera optical frame, or None.

        Hybrid depth: prefer the lidar cloud (lower noise at range, sees through
        no glass-ghosting), fall back to OAK stereo when too few cloud points
        project into the box (thin/distant objects, glass, partial FOV). The
        chosen depth is the NEAREST substantial surface, deprojected at the
        box-center pixel (fronto-parallel)."""
        W, H = self.preview_size
        x1, y1 = max(0, x1), max(0, y1)
        x2, y2 = min(W - 1, x2), min(H - 1, y2)
        if x2 <= x1 or y2 <= y1:
            return None
        z = None
        if self.use_lidar_depth:
            z = self._lidar_box_depth(x1, y1, x2, y2)
        if z is None:
            z = self._camera_box_depth(depth, x1, y1, x2, y2)
        if z is None:
            return None
        return deproject((x1 + x2) / 2.0, (y1 + y2) / 2.0, z,
                         self.fx, self.fy, self.cx, self.cy)

    def _camera_box_depth(self, depth, x1, y1, x2, y2):
        """Nearest-surface depth (m) from the OAK stereo image inside the box.

        We first sample only the central `depth_roi_frac` of the box (clean for
        solid objects like doors, avoids background bleed at the edges). If that
        region is mostly empty -- a glazed window, where the glass returns no
        depth and only the metal frame/mullions do -- we fall back to the full
        box, locking onto the frame/door plane."""
        bw, bh = x2 - x1, y2 - y1
        mx = int(bw * (1 - self.depth_roi_frac) / 2)
        my = int(bh * (1 - self.depth_roi_frac) / 2)
        roi = depth[y1 + my:y2 - my, x1 + mx:x2 - mx]
        valid = roi[roi > 0]
        if valid.size < self.min_plane_points:
            roi = depth[y1:y2, x1:x2]
            valid = roi[roi > 0]
        return self._near_surface_depth(valid)

    def _lidar_box_depth(self, x1, y1, x2, y2):
        """Nearest-surface depth (m) from lidar points projecting into the box,
        or None if no recent cloud / too few points hit the box."""
        with self.cloud_lock:
            cloud = self.latest_cloud
        if cloud is None:
            return None
        pts, frame_id = cloud
        try:
            tf = self.tf_buffer.lookup_transform(
                self.camera_optical_frame, frame_id, rclpy.time.Time(),
                timeout=Duration(seconds=0.05))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        R, t = transform_to_Rt(tf)
        pc = pts @ R.T + t                       # points in camera optical frame
        zc = pc[:, 2]
        front = zc > 0.05
        pc, zc = pc[front], zc[front]
        if zc.size == 0:
            return None
        u = self.fx * pc[:, 0] / zc + self.cx
        v = self.fy * pc[:, 1] / zc + self.cy
        inbox = (u >= x1) & (u <= x2) & (v >= y1) & (v <= y2)
        zsel = zc[inbox]
        if zsel.size < self.min_lidar_points:
            return None
        # reuse the stereo nearest-surface logic (it works in mm)
        return self._near_surface_depth(zsel * 1000.0, min_points=self.min_lidar_points)

    def to_map(self, p_cam, stamp):
        """Transform a camera-frame point (np.array) into the map frame, or None.

        Done in two decoupled hops through `odom`, each at its OWN latest time:
          1) camera_optical_frame -> odom   (the robot/URDF subtree)
          2) odom -> map                    (the SLAM map->odom link)
        A single chained map<-camera lookup at time 0 resolves to the *latest
        time common to the whole chain*; rtabmap publishes map->odom in bursts
        with a very short TF buffer, so that common time frequently lands a
        fraction of a second before map->odom's earliest buffered sample ->
        "extrapolation into the past". Splitting the hops lets each side use its
        own freshest transform; since odom is locally continuous the tiny time
        skew between the two is negligible for a stationary/slow robot."""
        ps = PointStamped()
        ps.header.frame_id = self.camera_optical_frame
        ps.header.stamp = rclpy.time.Time().to_msg()
        ps.point.x, ps.point.y, ps.point.z = float(p_cam[0]), float(p_cam[1]), float(p_cam[2])
        try:
            p_odom = self.tf_buffer.transform(ps, self.odom_frame, timeout=Duration(seconds=0.1))
            p_odom.header.stamp = rclpy.time.Time().to_msg()  # latest map->odom, not p_odom's time
            out = self.tf_buffer.transform(p_odom, self.map_frame, timeout=Duration(seconds=0.1))
            return np.array([out.point.x, out.point.y, out.point.z])
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF {self.camera_optical_frame}->{self.map_frame} failed: {e}",
                                   throttle_duration_sec=5.0)
            return None

    def lookup_frame_pose(self, source_frame, target_frame, stamp):
        """Pose of source_frame expressed in target_frame, or None if TF is unavailable."""
        try:
            # Time(0) = latest available transform; the SLAM map->odom link lags
            # wall-clock, so an exact-time lookup would extrapolate into the future.
            tf = self.tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time(), timeout=Duration(seconds=0.1)
            )
            t = tf.transform.translation
            q = tf.transform.rotation
            return {
                'x': t.x, 'y': t.y, 'z': t.z,
                'qx': q.x, 'qy': q.y, 'qz': q.z, 'qw': q.w,
            }
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None

    # ------------------------------------------------------------------
    def camera_callback(self):
        in_depth = self.q_depth.tryGet()
        if in_depth is not None:
            self.depth_frame = in_depth.getFrame()
        in_rgb = self.q_rgb.tryGet()
        if in_rgb is None or self.depth_frame is None:
            return
        frame = in_rgb.getCvFrame()
        depth = self.depth_frame
        stamp = self.get_clock().now().to_msg()

        try:
            results = self.model.predict(frame, imgsz=640, half=(self.device == 'cuda'),
                                         device=self.device, verbose=False)[0]
            with self.pose_lock:
                robot_pose = self.robot_pose.copy()
            with self.camera_pose_lock:
                camera_pose = self.camera_pose.copy()
            camera_pose_tf = self.lookup_frame_pose(self.camera_optical_frame, self.map_frame, stamp)
            if camera_pose_tf is not None:
                camera_pose = camera_pose_tf
            # robot_pose = robot base in the MAP frame (localized pose, matching
            # camera_pose and the detection centers). From TF, since the odom
            # topic subscription is starved by the inference timer.
            robot_pose_tf = self.lookup_frame_pose(self.robot_base_frame, self.map_frame, stamp)
            if robot_pose_tf is not None:
                robot_pose = robot_pose_tf

            annotated = results.plot() if self.display_window else None
            markers = MarkerArray()
            poses = PoseArray()
            poses.header.frame_id = self.map_frame
            poses.header.stamp = stamp
            rows = []

            for i, box in enumerate(results.boxes):
                cls_id = int(box.cls[0])
                name = results.names[cls_id]
                conf = float(box.conf[0])
                if conf < self.conf_threshold:
                    continue  # EVERY class above threshold is logged to CSV

                x1, y1, x2, y2 = map(int, box.xyxy[0].cpu().numpy())

                # 3D center is attempted for every object; columns stay blank if
                # depth is insufficient (the row is still written).
                ctr = ctr_m = None
                map_ok = False
                ctr = self.box_3d(depth, x1, y1, x2, y2)
                if ctr is not None:
                    ctr_m = self.to_map(ctr, stamp)
                    map_ok = ctr_m is not None

                rows.append(self._csv_row(stamp, i, name, conf, (x1, y1, x2, y2),
                                          ctr, map_ok, ctr_m, robot_pose, camera_pose))

                # Markers/poses for the nav stack: target classes only.
                if name.lower() in self.target_classes and map_ok:
                    self._add_outputs(markers, poses, name, i, ctr_m, stamp)
                if annotated is not None and name.lower() in self.target_classes:
                    self._annotate(annotated, (x1, y1, x2, y2), ctr, ctr_m)

            if rows:
                self._write_rows(rows)
            if markers.markers:
                self.marker_pub.publish(markers)
            if poses.poses:
                self.pose_pub.publish(poses)

            # FPS
            self.frame_count += 1
            el = time.time() - self.start_time
            if el > 1:
                self.current_fps = self.frame_count / el
                self.frame_count = 0
                self.start_time = time.time()
            if annotated is not None:
                cv2.putText(annotated, f"FPS: {self.current_fps:.1f}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                cv2.imshow("YOLO 3D Detections", annotated)
                if (cv2.waitKey(1) & 0xFF) == ord('q'):
                    # BaseException, so it is NOT swallowed by `except Exception`
                    # below; it propagates out of spin() to main's handler, which
                    # calls destroy_node() and actually closes the window.
                    raise KeyboardInterrupt
        except Exception as e:
            self.get_logger().error(f"Inference error: {e}")
            logging.error(f"Inference error: {e}")

    # ------------------------------------------------------------------
    def _csv_row(self, stamp, i, name, conf, bbox, ctr, map_ok, ctr_m,
                 robot_pose, camera_pose):
        z3 = lambda a: [f"{a[0]:.4f}", f"{a[1]:.4f}", f"{a[2]:.4f}"] if a is not None else ['', '', '']
        return ([datetime.now().isoformat(), i, name, f"{conf*100:.2f}",
                 f"{bbox[0]}", f"{bbox[1]}", f"{bbox[2]}", f"{bbox[3]}"]
                + z3(ctr)
                + [str(map_ok)] + z3(ctr_m)
                + [f"{robot_pose[k]:.6f}" for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')]
                + [f"{camera_pose[k]:.6f}" for k in ('x', 'y', 'z', 'qx', 'qy', 'qz', 'qw')])

    def _write_rows(self, rows):
        try:
            with open(self.csv_filename, 'a', newline='') as f:
                csv.writer(f).writerows(rows)
        except Exception as e:
            self.get_logger().error(f"CSV writing error: {e}")

    def _add_outputs(self, markers, poses, name, idx, ctr_m, stamp):
        m = Marker()
        m.header.frame_id = self.map_frame
        m.header.stamp = stamp
        m.ns = name
        m.id = idx
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x, m.pose.position.y, m.pose.position.z = map(float, ctr_m)
        m.pose.orientation.w = 1.0
        m.scale.x = m.scale.y = m.scale.z = 0.15
        m.color.a = 0.8
        m.color.r, m.color.g, m.color.b = (0.0, 0.6, 1.0) if name.lower() == 'window' else (1.0, 0.4, 0.0)
        m.lifetime = Duration(seconds=5.0).to_msg()
        markers.markers.append(m)
        p = Pose()
        p.position.x, p.position.y, p.position.z = map(float, ctr_m)
        p.orientation.w = 1.0
        poses.poses.append(p)

    def _annotate(self, img, bbox, ctr, ctr_m):
        """Draw the box-center 3D coord in the map frame at the box center.

        Shows 'no depth' when the object is beyond the configured depth/range
        bound or depth is otherwise insufficient. If camera depth exists but the
        map transform is unavailable, shows 'no map tf' for easier debugging.
        """
        x1, y1, x2, y2 = bbox
        H, W = img.shape[:2]
        px, py = (x1 + x2) // 2, (y1 + y2) // 2
        cv2.circle(img, (px, py), 4, (0, 255, 255), -1)
        if ctr_m is not None:
            text = f"map({ctr_m[0]:.2f},{ctr_m[1]:.2f},{ctr_m[2]:.2f})"
        elif ctr is None:
            text = "no depth"
        else:
            text = "no map tf"
        tx = int(min(max(px + 6, 2), W - 150))
        ty = int(min(max(py - 6, 12), H - 4))
        cv2.putText(img, text, (tx, ty),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1, cv2.LINE_AA)

    def destroy_node(self):
        self.get_logger().info("Shutting down YOLO 3D Detection Node")
        if hasattr(self, 'cloud_executor'):
            self.cloud_executor.shutdown()
            self.cloud_node.destroy_node()
        if self.display_window:
            cv2.destroyAllWindows()
            for _ in range(5):      # flush GUI events so the window actually closes
                cv2.waitKey(1)
        if hasattr(self, 'oak_device'):
            self.oak_device.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YOLODetectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        # destroy unconditionally: pressing 'q' calls rclpy.shutdown() first, so
        # rclpy.ok() is already False here -- gating on it would skip cleanup and
        # leave the OpenCV window open.
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
