#!/usr/bin/env python3
import html
import json
import sys
import os
# Make the sibling UI_utils package importable whether this file is run from the
# source tree (symlink install) or from lib/robo_drill (copy install).
sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
import shlex
import subprocess
import signal
import socket
import re
import time
import rclpy
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy
from PyQt5.QtWidgets import *
from PyQt5.QtCore import QTimer, QProcess, Qt
from PyQt5.QtGui import QFontMetrics, QIcon, QPixmap, QPalette, QColor
from PyQt5.QtWidgets import QApplication
from ament_index_python.packages import get_package_share_directory
from UI_utils.qtermwidget_wrapper import QTermWidget
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

def _detect_system_dark():
    """Return True if the GNOME/Ubuntu system colour-scheme is set to dark."""
    try:
        out = subprocess.run(
            ['gsettings', 'get', 'org.gnome.desktop.interface', 'color-scheme'],
            capture_output=True, text=True, timeout=2
        ).stdout
        return 'dark' in out.lower()
    except Exception:
        return False

def _make_dark_palette():
    p = QPalette()
    c = QColor
    p.setColor(QPalette.Window,          c('#1c2128'))
    p.setColor(QPalette.WindowText,      c('#cdd9e5'))
    p.setColor(QPalette.Base,            c('#2d333b'))
    p.setColor(QPalette.AlternateBase,   c('#22272e'))
    p.setColor(QPalette.ToolTipBase,     c('#2d333b'))
    p.setColor(QPalette.ToolTipText,     c('#cdd9e5'))
    p.setColor(QPalette.Text,            c('#cdd9e5'))
    p.setColor(QPalette.Button,          c('#2d333b'))
    p.setColor(QPalette.ButtonText,      c('#cdd9e5'))
    p.setColor(QPalette.BrightText,      c('#ffffff'))
    p.setColor(QPalette.Link,            c('#539bf5'))
    p.setColor(QPalette.Highlight,       c('#1f6feb'))
    p.setColor(QPalette.HighlightedText, c('#ffffff'))
    p.setColor(QPalette.Disabled, QPalette.WindowText, c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.Text,       c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.ButtonText, c('#768390'))
    p.setColor(QPalette.Disabled, QPalette.Base,       c('#22272e'))
    p.setColor(QPalette.Disabled, QPalette.Button,     c('#22272e'))
    return p

class RobotControlUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self._dark_theme = _detect_system_dark()
 
        # Initialize ROS only if not already initialized
        if not rclpy.ok():
            rclpy.init()
            
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
 
        self.node = rclpy.create_node('robot_control_ui')
        self.emergency_stop_publisher = self.node.create_publisher(Bool, '/emergency_stop', qos)
        self.emergency_stop_subscriber = self.node.create_subscription(Bool, '/emergency_stop', self._on_emergency_stop_state, qos)
        # Persistent publishers for emergency stop topics (keyed by topic name), created on demand
        self._emergency_stop_publishers = {}
        # Publisher for jogging the on-board gantry (Gantry Control tab).
        # Order matches gantry_position_controller joints: [stage1_lift, stage2_lift, stage3_rotate, stage4_horizontal].
        self.gantry_cmd_publisher = self.node.create_publisher(
            Float64MultiArray, '/gantry_position_controller/commands', 10
        )

        # Latched parking-active flag from the base controller. Used to wait for the
        # chassis-parking maneuver to finish before shutting down the base robot.
        self._parking_active = False
        parking_active_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.parking_active_subscriber = self.node.create_subscription(
            Bool,
            '/sim_controller/parking_active',
            self._on_parking_active,
            parking_active_qos,
        )

        # Track processes and their associated buttons
        self.process_map = {}
        self.button_map = {}
 
        # Store cleared status text for restore functionality
        self.cleared_status_backup = None
        self.base_cleared_status_backup = None
        self.fsm_cleared_status_backup = None

        # FSM processes
        self.fsm_launch_process = None
        self.fsm_node_process = None

        # FSM log buffer (html, plain_text, add_newline) for filter rebuilds
        self.fsm_log_entries = []
        self.fsm_log_backup = []
        
        # Store process list for kill functionality
        self.current_process_list = []
        self.ps_output_accumulator = ""
        
        # Emergency stop UI state
        self.emergency_stop_active = False
        self.freedrive_active = False
        self.freedrive_transition_in_progress = False
        self.freedrive_transition_target_active = False
        self.freedrive_trajectory_controller = None
        self.freedrive_controller_manager = '/controller_manager'
 
        # Robot dashboard connection
        self.robot_socket = None
        self.robot_host = '192.168.56.101'
        self.robot_port = 29999
 
        # Action client for canceling trajectory goals
        self.action_client = ActionClient(
            self.node, 
            FollowJointTrajectory, 
            '/arm/joint_trajectory_controller/follow_joint_trajectory'
        )
        self.current_goal_handle = None
 
        self.setWindowTitle("Pokeye Robot Control Panel")
        self.setGeometry(100, 100, 1200, 700)
 
        # Set window icon
        try:
            pkg_share = get_package_share_directory('robo_drill')
            icon_path = os.path.join(pkg_share, 'resource', 'robot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
        except Exception:
            # Fallback to source directory if package not found
            icon_path = os.path.join(os.path.dirname(os.path.dirname(__file__)), 'resource', 'robot_icon.png')
            if os.path.exists(icon_path):
                self.setWindowIcon(QIcon(icon_path))
 
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
 
        # Theme toggle button
        theme_bar = QHBoxLayout()
        theme_bar.addStretch()
        self.btn_theme_toggle = QPushButton("☀ Light Theme")
        self.btn_theme_toggle.setFixedWidth(120)
        self.btn_theme_toggle.clicked.connect(self._toggle_theme)
        theme_bar.addWidget(self.btn_theme_toggle)
        main_layout.addLayout(theme_bar)

        # Create tab widget for Arm Control and Base Control
        tabs = QTabWidget()
        main_layout.addWidget(tabs)

        self.tabs = tabs
        # Generic per-widget log buffers and filter inputs
        self.tab_log_entries: dict = {}   # id(widget) -> list of (html, plain_text, add_newline)
        self.tab_log_backups: dict = {}   # id(widget) -> backup list for clear/restore
        self.tab_filter_inputs: dict = {} # id(widget) -> QLineEdit
        # ===== BASE CONTROL TAB =====
        base_tab = QWidget()
        base_tab_layout = QVBoxLayout(base_tab)
 
        # Horizontal layout for control boxes side by side
        base_boxes_layout = QHBoxLayout()
        base_tab_layout.addLayout(base_boxes_layout)
 
        # Simulation parameter selector (at top)
        sim_param_layout = QHBoxLayout()
        sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.sim_mode_combo = QComboBox()
        self.sim_mode_combo.addItems(['false', 'true'])
        self.sim_mode_combo.currentTextChanged.connect(self._update_headless_visibility)
        sim_param_layout.addWidget(self.sim_mode_combo)
        sim_param_layout.addWidget(QLabel("Controller Type:"))
        self.controller_type_combo = QComboBox()
        self.controller_type_combo.addItems(['omni', 'diff'])
        sim_param_layout.addWidget(self.controller_type_combo)
        self.base_headless_label = QLabel("Headless:")
        self.base_headless_combo = QComboBox()
        self.base_headless_combo.addItems(['false', 'true'])
        self.base_headless_combo.setCurrentText('true')
        sim_param_layout.addWidget(self.base_headless_label)
        sim_param_layout.addWidget(self.base_headless_combo)
        sim_param_layout.addStretch()
        base_tab_layout.insertLayout(0, sim_param_layout)
 
        # Map and Localization Box
        mapping_loc_box = QGroupBox("Map & Localization")
        mapping_loc_layout = QVBoxLayout()
        mapping_loc_box.setLayout(mapping_loc_layout)

        self.btn_launch_base_robot = QPushButton("Launch Base Robot")
        self.btn_launch_base_robot.clicked.connect(self.toggle_base_control_launch)
        self.btn_launch_base_robot.setToolTip(
            "ros2 launch robo_drill platform.launch.py sim:=<mode> mode:=base "
            "controller_type:=<type> odom_tf_from_controller:=true "
            "publish_controller_odom_tf:=true launch_rviz:=true "
            "headless:=<true/false>"
        )
        mapping_loc_layout.addWidget(self.btn_launch_base_robot)

        # Launch Localization button
        self.btn_launch_localization = QPushButton("Start Localization")
        self.btn_launch_localization.clicked.connect(lambda: self.toggle_localization(controller_type_combo=self.controller_type_combo, headless_combo=self.base_headless_combo))
        self.btn_launch_localization.setToolTip("ros2 launch robo_drill move_robot.launch.py sim:=<mode> mode:=base controller_type:=<type> headless:=<true/false>")
        mapping_loc_layout.addWidget(self.btn_launch_localization)
 
        # View map button
        self.btn_view_map = QPushButton("View map")
        self.btn_view_map.clicked.connect(self.toggle_view_map)
        self.btn_view_map.setToolTip("rtabmap-databaseViewer rtabmap.db")
        mapping_loc_layout.addWidget(self.btn_view_map)
 
        mapping_loc_layout.addStretch()
        base_boxes_layout.addWidget(mapping_loc_box)
 
        # Navigation Box
        nav_box = QGroupBox("Navigation")
        nav_layout = QVBoxLayout()
        nav_box.setLayout(nav_layout)

        # Launch Nav2 button
        self.btn_launch_nav2 = QPushButton("Launch Nav2")
        self.btn_launch_nav2.clicked.connect(lambda: self.toggle_nav2(controller_type_combo=self.controller_type_combo))
        self.btn_launch_nav2.setToolTip("ros2 launch robo_drill navigation_launch.py use_sim_time:=<mode> controller_type:=<type>")
        nav_layout.addWidget(self.btn_launch_nav2)

        nav_layout.addStretch()
        base_boxes_layout.addWidget(nav_box)
 
        # Troubleshooting Box
        troubleshooting_box = QGroupBox("Troubleshooting")
        troubleshooting_layout = QVBoxLayout()
        troubleshooting_box.setLayout(troubleshooting_layout)
 
        # PS AUX button
        btn_ps_ros = QPushButton("List ROS2 Processes")
        btn_ps_ros.clicked.connect(self.run_ps_ros)
        btn_ps_ros.setToolTip("ps aux | grep -E 'ros2|robot'")
        troubleshooting_layout.addWidget(btn_ps_ros)
 
        # List Controllers button
        btn_list_base_controllers = QPushButton("List Controllers")
        btn_list_base_controllers.clicked.connect(self.run_list_base_controllers)
        btn_list_base_controllers.setToolTip("List controllers for all detected controller_manager nodes")
        troubleshooting_layout.addWidget(btn_list_base_controllers)
 
        # Launch RQT button
        self.btn_launch_rqt = QPushButton("Start RQT")
        self.btn_launch_rqt.clicked.connect(self.toggle_rqt)
        self.btn_launch_rqt.setToolTip("rqt")
        troubleshooting_layout.addWidget(self.btn_launch_rqt)
 
        troubleshooting_layout.addStretch()
        base_boxes_layout.addWidget(troubleshooting_box)
 
        # Terminal and Status for Base Control tab (side by side)
        base_terminal_status_layout = QHBoxLayout()
        
        # Left side: Terminal
        self.base_terminal = QTermWidget()
        base_terminal_status_layout.addWidget(self.base_terminal)
        
        # Right side: Status display
        self.base_status_text = QTextEdit()
        self.base_status_text.setReadOnly(True)
        self.base_status_text.setAcceptRichText(True)
        self.base_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        # Generic shared handlers default to self.status_text (previously the Arm tab's
        # widget). With the Arm tab gone, point that default at the Base tab's status log.
        self.status_text = self.base_status_text

        # Set tab stops to 8 characters (standard terminal width)
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.base_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.base_status_text.setTabStopDistance(tab_width)
        base_terminal_status_layout.addWidget(self.base_status_text)

        base_status_header = QHBoxLayout()
        base_status_header.addWidget(QLabel("Terminal + Status:"))
        base_status_header.addStretch()

        # Filter bar
        base_status_header.addWidget(QLabel("Filter:"))
        self.base_search_input = QLineEdit()
        self.base_search_input.setPlaceholderText("Filter output lines...")
        self.base_search_input.setMaximumWidth(200)
        self.base_search_input.textChanged.connect(lambda: self._log_apply_filter(self.base_status_text))
        base_status_header.addWidget(self.base_search_input)
        self.tab_filter_inputs[id(self.base_status_text)] = self.base_search_input

        self.btn_restore_base_status = QPushButton("Restore")
        self.btn_restore_base_status.clicked.connect(self.restore_base_status)
        self.btn_restore_base_status.setMaximumWidth(80)
        self.btn_restore_base_status.setVisible(False)
        base_status_header.addWidget(self.btn_restore_base_status)

        btn_clear_base_status = QPushButton("Clear")
        btn_clear_base_status.clicked.connect(self.clear_base_status)
        btn_clear_base_status.setMaximumWidth(80)
        base_status_header.addWidget(btn_clear_base_status)
        base_tab_layout.addLayout(base_status_header)

        base_tab_layout.addLayout(base_terminal_status_layout)
 
        tabs.addTab(base_tab, "Base Control")
 
        # ===== GANTRY CONTROL TAB (was Joint Control) =====
        # Jogs the on-board 3-stage linear gantry + end rotation by publishing
        # position targets to /gantry_position_controller/commands.
        joint_tab = QWidget()
        joint_tab_layout = QVBoxLayout(joint_tab)

        gantry_control_box = QGroupBox("Gantry Position Control")
        gantry_control_layout = QVBoxLayout()
        gantry_control_box.setLayout(gantry_control_layout)

        gantry_control_layout.addWidget(QLabel(
            "<b>Gantry stage targets</b> - published to "
            "<code>/gantry_position_controller/commands</code>"
        ))

        self.gantry_joint_names = ['stage1_lift_joint', 'stage2_lift_joint', 'stage3_rotate_joint', 'stage4_horizontal_joint']
        self.gantry_joint_labels = {
            'stage1_lift_joint': 'Stage 1 - Lift 1 / vertical (m)',
            'stage2_lift_joint': 'Stage 2 - Lift 2 / vertical (m)',
            'stage3_rotate_joint': 'Stage 3 - End Rotation (rad)',
            'stage4_horizontal_joint': 'Stage 4 - Lateral / horizontal Y (m)',
        }
        self.gantry_joint_limits = {
            'stage1_lift_joint': (-0.47, 0.40),
            'stage2_lift_joint': (0.0, 1.0),
            'stage3_rotate_joint': (-3.1416, 3.1416),
            'stage4_horizontal_joint': (-0.196, 0.196),
        }
        self.gantry_scale = 1000  # slider integer scale

        self.gantry_inputs = {}
        self.gantry_sliders = {}
        for joint_name in self.gantry_joint_names:
            row = QHBoxLayout()
            label = QLabel(self.gantry_joint_labels[joint_name])
            label.setMinimumWidth(170)
            row.addWidget(label)

            min_limit, max_limit = self.gantry_joint_limits[joint_name]
            slider = QSlider(Qt.Horizontal)
            slider.setRange(int(min_limit * self.gantry_scale), int(max_limit * self.gantry_scale))
            slider.setValue(0)
            row.addWidget(slider)

            spin = QDoubleSpinBox()
            spin.setRange(min_limit, max_limit)
            spin.setValue(0.0)
            spin.setDecimals(3)
            spin.setSingleStep(0.01)
            spin.setMaximumWidth(120)
            row.addWidget(spin)

            slider.valueChanged.connect(lambda val, s=spin: s.setValue(val / self.gantry_scale))
            spin.valueChanged.connect(lambda val, sl=slider: sl.setValue(int(val * self.gantry_scale)))

            self.gantry_inputs[joint_name] = spin
            self.gantry_sliders[joint_name] = slider
            gantry_control_layout.addLayout(row)

        gantry_button_layout = QHBoxLayout()
        self.btn_send_gantry = QPushButton("Send Gantry Command")
        self.btn_send_gantry.clicked.connect(self.send_gantry_command)
        self.btn_send_gantry.setToolTip(
            "ros2 topic pub --once /gantry_position_controller/commands "
            "std_msgs/msg/Float64MultiArray \"{data: [Z, Y, X, ROT]}\""
        )
        gantry_button_layout.addWidget(self.btn_send_gantry)

        self.btn_home_gantry = QPushButton("Home (0, 0, 0)")
        self.btn_home_gantry.clicked.connect(self.home_gantry)
        gantry_button_layout.addWidget(self.btn_home_gantry)
        gantry_control_layout.addLayout(gantry_button_layout)

        joint_tab_layout.addWidget(gantry_control_box)

        # Terminal and Status for Gantry Control tab (side by side)
        joint_terminal_status_layout = QHBoxLayout()
        self.joint_terminal = QTermWidget()
        joint_terminal_status_layout.addWidget(self.joint_terminal)
        self.joint_status_text = QTextEdit()
        self.joint_status_text.setReadOnly(True)
        self.joint_status_text.setAcceptRichText(True)
        self.joint_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.joint_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.joint_status_text.setTabStopDistance(tab_width)
        joint_terminal_status_layout.addWidget(self.joint_status_text)

        joint_status_header = QHBoxLayout()
        joint_status_header.addWidget(QLabel("Gantry command log + terminal"))
        joint_status_header.addStretch()
        btn_clear_joint_status = QPushButton("Clear")
        btn_clear_joint_status.clicked.connect(self.clear_joint_status)
        btn_clear_joint_status.setMaximumWidth(80)
        joint_status_header.addWidget(btn_clear_joint_status)
        joint_tab_layout.addLayout(joint_status_header)
        joint_tab_layout.addLayout(joint_terminal_status_layout)
        
        tabs.addTab(joint_tab, "Gantry Control")
        # ===== FULL CONTROL TAB =====
        # For robo_drill, "full" == base platform + on-board gantry manipulator.
        # The gantry is part of the robot description, so full mode simply brings
        # up the whole platform with mode:=full. There is no separate arm/MoveIt stack.
        full_control_tab = QWidget()
        full_control_tab_layout = QVBoxLayout(full_control_tab)

        # Simulation parameter selector (at top)
        full_control_sim_param_layout = QHBoxLayout()
        full_control_sim_param_layout.addWidget(QLabel("Simulation Mode:"))
        self.full_control_sim_mode_combo = QComboBox()
        self.full_control_sim_mode_combo.addItems(['false', 'true'])
        self.full_control_sim_mode_combo.currentTextChanged.connect(self._update_full_control_sim_mode)
        full_control_sim_param_layout.addWidget(self.full_control_sim_mode_combo)
        full_control_sim_param_layout.addWidget(QLabel("Controller Type:"))
        self.full_control_controller_type_combo = QComboBox()
        self.full_control_controller_type_combo.addItems(['omni', 'diff'])
        full_control_sim_param_layout.addWidget(self.full_control_controller_type_combo)
        self.full_control_headless_label = QLabel("Headless:")
        self.full_control_headless_combo = QComboBox()
        self.full_control_headless_combo.addItems(['false', 'true'])
        self.full_control_headless_combo.setCurrentText('true')
        self.full_control_headless_combo.currentTextChanged.connect(
            self._update_full_control_launch_tooltip
        )
        full_control_sim_param_layout.addWidget(self.full_control_headless_label)
        full_control_sim_param_layout.addWidget(self.full_control_headless_combo)
        full_control_sim_param_layout.addStretch()
        full_control_tab_layout.addLayout(full_control_sim_param_layout)

        # Horizontal layout for control boxes side by side
        full_control_boxes_layout = QHBoxLayout()
        full_control_tab_layout.addLayout(full_control_boxes_layout)

        # Localization, Navigation Box
        full_control_mapping_box = QGroupBox("Localization & Navigation")
        full_control_mapping_layout = QVBoxLayout()
        full_control_mapping_box.setLayout(full_control_mapping_layout)

        self.btn_full_control_launch = QPushButton("Launch Full Robot")
        self.btn_full_control_launch.clicked.connect(self.toggle_full_control_launch)
        full_control_mapping_layout.addWidget(self.btn_full_control_launch)

        self.btn_full_control_localization = QPushButton("Start Localization")
        self.btn_full_control_localization.clicked.connect(
            lambda: self.toggle_localization(
                mode='full',
                button=self.btn_full_control_localization,
                sim_combo=self.full_control_sim_mode_combo,
                controller_type_combo=self.full_control_controller_type_combo,
                headless_combo=self.full_control_headless_combo,
            )
        )
        self.btn_full_control_localization.setToolTip("ros2 launch robo_drill move_robot.launch.py sim:=<mode> mode:=full controller_type:=<type> headless:=<true/false>")
        full_control_mapping_layout.addWidget(self.btn_full_control_localization)

        self.btn_full_control_nav2 = QPushButton("Launch Nav2")
        self.btn_full_control_nav2.clicked.connect(lambda: self.toggle_nav2(mode='full', button=self.btn_full_control_nav2, sim_combo=self.full_control_sim_mode_combo, controller_type_combo=self.full_control_controller_type_combo))
        self.btn_full_control_nav2.setToolTip("ros2 launch robo_drill navigation_launch.py use_sim_time:= controller_type:=<type>")
        full_control_mapping_layout.addWidget(self.btn_full_control_nav2)

        # Emergency stop button for Full Control tab
        full_control_mapping_layout.addWidget(QLabel(""))  # Spacer
        self.btn_full_control_emergency_stop = QPushButton("EMERGENCY STOP (Click to Activate)")
        self.btn_full_control_emergency_stop.setCheckable(True)
        self.btn_full_control_emergency_stop.setStyleSheet("background-color: red; color: white; font-weight: bold;")
        self.btn_full_control_emergency_stop.clicked.connect(lambda: self.emergency_stop(context='full'))
        self.btn_full_control_emergency_stop.setToolTip("Toggle emergency stop and cancel goals when activating")
        full_control_mapping_layout.addWidget(self.btn_full_control_emergency_stop)

        full_control_mapping_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_mapping_box, 1)

        # Troubleshooting Box (Full Control)
        full_control_troubleshooting_box = QGroupBox("Troubleshooting")
        full_control_troubleshooting_layout = QVBoxLayout()
        full_control_troubleshooting_box.setLayout(full_control_troubleshooting_layout)

        self.btn_full_control_view_map = QPushButton("View map")
        self.btn_full_control_view_map.clicked.connect(
            lambda: self.toggle_view_map(mode='full', button=self.btn_full_control_view_map)
        )
        self.btn_full_control_view_map.setToolTip("rtabmap-databaseViewer rtabmap.db")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_view_map)

        self.btn_full_control_list_controllers = QPushButton("List Controllers")
        self.btn_full_control_list_controllers.clicked.connect(self.run_list_full_controllers)
        self.btn_full_control_list_controllers.setToolTip("List controllers for all detected controller_manager nodes")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_list_controllers)

        self.btn_full_control_rqt = QPushButton("Start RQT")
        self.btn_full_control_rqt.clicked.connect(
            lambda: self.toggle_rqt(mode='full', button=self.btn_full_control_rqt)
        )
        self.btn_full_control_rqt.setToolTip("rqt")
        full_control_troubleshooting_layout.addWidget(self.btn_full_control_rqt)

        # List ROS2 Processes button
        btn_full_control_ps_ros = QPushButton("List ROS2 Processes")
        btn_full_control_ps_ros.clicked.connect(self.run_full_control_ps_ros)
        btn_full_control_ps_ros.setToolTip("ps aux | grep -E 'ros2|robot'")
        full_control_troubleshooting_layout.addWidget(btn_full_control_ps_ros)

        # ROS2 Topics Section
        full_control_troubleshooting_layout.addWidget(QLabel("\nROS2 Topics:"))
        topic_selector_layout = QHBoxLayout()
        self.topics_combo = QComboBox()
        self.topics_combo.setEditable(True)
        self.topics_combo.setMinimumWidth(200)
        topic_selector_layout.addWidget(self.topics_combo)
        btn_refresh_topics = QPushButton("Refresh")
        btn_refresh_topics.clicked.connect(self.refresh_topics_list)
        btn_refresh_topics.setToolTip("Refresh available ROS2 topics")
        btn_refresh_topics.setMaximumWidth(80)
        topic_selector_layout.addWidget(btn_refresh_topics)
        full_control_troubleshooting_layout.addLayout(topic_selector_layout)
        topic_actions_layout = QHBoxLayout()
        btn_topic_bw = QPushButton("Bandwidth")
        btn_topic_bw.clicked.connect(self.check_topic_bandwidth)
        btn_topic_bw.setToolTip("Check topic bandwidth (ros2 topic bw)")
        topic_actions_layout.addWidget(btn_topic_bw)
        btn_topic_hz = QPushButton("Frequency")
        btn_topic_hz.clicked.connect(self.check_topic_frequency)
        btn_topic_hz.setToolTip("Check topic frequency (ros2 topic hz)")
        topic_actions_layout.addWidget(btn_topic_hz)
        btn_topic_echo = QPushButton("Echo Once")
        btn_topic_echo.clicked.connect(self.echo_topic_once)
        btn_topic_echo.setToolTip("Echo topic once (ros2 topic echo --once)")
        topic_actions_layout.addWidget(btn_topic_echo)
        full_control_troubleshooting_layout.addLayout(topic_actions_layout)
        self.topic_info_display = QTextEdit()
        self.topic_info_display.setReadOnly(True)
        self.topic_info_display.setMaximumHeight(60)
        self.topic_info_display.setPlaceholderText("Topic bandwidth and frequency info will appear here...")
        self.topic_info_display.setStyleSheet("background-color: #f6f8fa; color: #1f2328; border: 1px solid #d0d7de; font-family: 'Courier New', monospace; padding: 4px;")
        full_control_troubleshooting_layout.addWidget(self.topic_info_display)

        # Process Kill Section
        full_control_troubleshooting_layout.addWidget(QLabel("\nProcess Management:"))
        process_selector_layout = QHBoxLayout()
        self.process_combo = QComboBox()
        self.process_combo.setEditable(False)
        self.process_combo.setMinimumWidth(150)
        self.process_combo.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        self.process_combo.setPlaceholderText("Select process to kill...")
        process_selector_layout.addWidget(self.process_combo, 1)
        btn_kill_process = QPushButton("Kill Process")
        btn_kill_process.clicked.connect(self.kill_selected_process)
        btn_kill_process.setToolTip("Kill the selected process using kill -9")
        btn_kill_process.setMaximumWidth(120)
        btn_kill_process.setStyleSheet("background-color: #d73a49; color: white; font-weight: bold;")
        process_selector_layout.addWidget(btn_kill_process, 0)
        btn_kill_all = QPushButton("Kill All")
        btn_kill_all.clicked.connect(self.kill_all_processes)
        btn_kill_all.setToolTip("Kill all detected processes except UI and ros2 daemon")
        btn_kill_all.setMaximumWidth(120)
        btn_kill_all.setStyleSheet("background-color: #8B0000; color: white; font-weight: bold;")
        process_selector_layout.addWidget(btn_kill_all, 0)
        full_control_troubleshooting_layout.addLayout(process_selector_layout)

        full_control_troubleshooting_layout.addStretch()
        full_control_boxes_layout.addWidget(full_control_troubleshooting_box, 1)

        # Status display for Full Control tab
        full_control_terminal_status_layout = QHBoxLayout()
        self.full_control_status_text = QTextEdit()
        self.full_control_status_text.setReadOnly(True)
        self.full_control_status_text.setAcceptRichText(True)
        self.full_control_status_text.setStyleSheet("background-color: #22272e; color: #adbac7; border: 1px solid #444c56; font-family: 'Courier New', monospace; white-space: pre;")
        from PyQt5.QtGui import QFontMetrics
        font_metrics = QFontMetrics(self.full_control_status_text.font())
        tab_width = font_metrics.horizontalAdvance(' ') * 8
        self.full_control_status_text.setTabStopDistance(tab_width)
        full_control_terminal_status_layout.addWidget(self.full_control_status_text)

        # Status header with search functionality
        full_control_status_header = QHBoxLayout()
        full_control_status_header.addWidget(QLabel("Full Control - Status"))
        full_control_status_header.addStretch()
        full_control_status_header.addWidget(QLabel("Filter:"))
        self.full_control_search_input = QLineEdit()
        self.full_control_search_input.setPlaceholderText("Filter output lines...")
        self.full_control_search_input.setMaximumWidth(200)
        self.full_control_search_input.textChanged.connect(lambda: self._log_apply_filter(self.full_control_status_text))
        full_control_status_header.addWidget(self.full_control_search_input)
        self.tab_filter_inputs[id(self.full_control_status_text)] = self.full_control_search_input
        self.btn_restore_full_control_status = QPushButton("Restore")
        self.btn_restore_full_control_status.clicked.connect(self.restore_full_control_status)
        self.btn_restore_full_control_status.setMaximumWidth(80)
        self.btn_restore_full_control_status.setVisible(False)
        full_control_status_header.addWidget(self.btn_restore_full_control_status)
        btn_clear_full_control_status = QPushButton("Clear")
        btn_clear_full_control_status.clicked.connect(self.clear_full_control_status)
        btn_clear_full_control_status.setMaximumWidth(80)
        full_control_status_header.addWidget(btn_clear_full_control_status)

        full_control_tab_layout.addLayout(full_control_status_header)
        full_control_tab_layout.addLayout(full_control_terminal_status_layout)
        # Add Full Control tab after Joint Control
        tabs.addTab(full_control_tab, "Full Control")
        tabs.addTab(self._create_fsm_tab(), "FSM")

        # Connect tab change signal to check joint states when Joint Control tab is activated
        tabs.currentChanged.connect(lambda index: self._on_tab_changed(index, tabs))
        
        # Timer for ROS spinning
        self.timer = QTimer()
        self.timer.timeout.connect(self._spin_ros)
        self.timer.start(100)  # 10 Hz
        
        # Initial UI update for consistent visuals
        self._update_emergency_stop_button_ui()
        self._update_freedrive_button_ui()
        self.BASE_TAB_INDEX = 0
        self.JOINT_TAB_INDEX = 1
        self.FULL_CONTROL_TAB_INDEX = 2
        self.FSM_TAB_INDEX = 3
        self._active_planner_context = 'full'
        self._update_full_control_init_box_state()
        self._update_headless_visibility()
        self._update_full_control_planner_constraints()
        
        # Initial topics list refresh
        QTimer.singleShot(1000, self.refresh_topics_list)  # Delay 1s to ensure ROS is ready

        self._apply_theme()

    def _toggle_theme(self):
        self._dark_theme = not self._dark_theme
        self._apply_theme()

    def _apply_theme(self):
        dark = self._dark_theme
        app = QApplication.instance()
        if dark:
            app.setPalette(_make_dark_palette())
            log_style = (
                "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
                "font-family: 'Courier New', monospace;"
            )
            info_style = (
                "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
                "font-family: 'Courier New', monospace; padding: 4px;"
            )
        else:
            app.setPalette(app.style().standardPalette())
            log_style = (
                "background-color: #f0f0f0; color: #1f2328; border: 1px solid #d0d7de; "
                "font-family: 'Courier New', monospace;"
            )
            info_style = (
                "background-color: #f0f0f0; color: #1f2328; border: 1px solid #d0d7de; "
                "font-family: 'Courier New', monospace; padding: 4px;"
            )

        for attr in ('status_text', 'base_status_text', 'joint_status_text',
                     'full_control_status_text', 'gpr_status_text'):
            if hasattr(self, attr):
                getattr(self, attr).setStyleSheet(log_style)

        if hasattr(self, 'topic_info_display'):
            self.topic_info_display.setStyleSheet(info_style)

        if hasattr(self, 'btn_theme_toggle'):
            self.btn_theme_toggle.setText("☀ Light Theme" if dark else "☾ Dark Theme")

    def _update_full_control_sim_mode(self):
        """Refresh Full Control state when its simulation mode changes."""
        self._update_full_control_planner_constraints()
        if hasattr(self, "btn_full_control_vnc"):
            self.btn_full_control_vnc.setEnabled(
                self.full_control_sim_mode_combo.currentText() == 'false'
            )

    def _is_robot_bringup_process(self, process_key):
        """Return whether a process key belongs to a robot bringup launch."""
        return process_key in {'mobile_platform', 'full_mobile_manipulator'}

    def _is_base_tab_state_process(self, process_key):
        """Return whether a process should lock the Base Control tab context."""
        return process_key in {'mobile_platform', 'localization', 'nav2'}

    def _get_base_process_start_text(self, process_key, name):
        """Return the idle button label for base/full-control launch buttons."""
        if process_key == 'full_mobile_manipulator':
            return 'Launch Full Robot'
        if process_key == 'mobile_platform':
            return 'Launch Base Robot'
        return f"Start {name}"

    def _get_base_process_stop_text(self, process_key, name):
        """Return the running button label for base/full-control launch buttons."""
        if process_key == 'full_mobile_manipulator':
            return 'Stop Full Robot'
        if process_key == 'mobile_platform':
            return 'Stop Base Robot'
        return f"Stop {name}"

    def _get_full_control_launch_file(self):
        """Full Control brings up the whole robo_drill robot (base + gantry manipulator)."""
        return 'pokeye_mobile_manipulator.launch.py'

    def _update_full_control_launch_tooltip(self):
        """Keep the Full Control launch tooltip aligned with the selected launch file."""
        if not hasattr(self, 'btn_full_control_launch'):
            return

        headless = (
            self.full_control_headless_combo.currentText()
            if hasattr(self, 'full_control_headless_combo')
            else 'false'
        )
        tooltip = (
            "ros2 launch robo_drill pokeye_mobile_manipulator.launch.py "
            f"sim:=<mode> mode:=full controller_type:=<type> "
            f"publish_controller_odom_tf:=true launch_rviz:=true headless:={headless}"
        )
        self.btn_full_control_launch.setToolTip(tooltip)

    def _update_full_control_planner_constraints(self):
        """Refresh Full Control planner-dependent UI elements."""
        planner = self._get_planner_backend(context='full')

        # Disable Reset Planner button when backend is moveit
        if hasattr(self, 'btn_full_control_reset_planner'):
            self.btn_full_control_reset_planner.setEnabled(planner != 'moveit')

        # Update Joint Control tab state based on the active planner tab
        self._update_joint_control_tab_state()

        self._update_moveit_option_visibility()
        self._update_full_control_init_box_state()
        self._update_headless_visibility()
        self._update_full_control_launch_tooltip()

    def _update_moveit_option_visibility(self):
        """Show MoveIt pipeline/planner selectors only when they are relevant."""
        full_backend_is_moveit = self._get_planner_backend(context='full') == 'moveit'
        full_pipeline_is_pilz = (
            hasattr(self, 'full_control_moveit_pipeline_combo')
            and self.full_control_moveit_pipeline_combo.currentText()
            == 'pilz_industrial_motion_planner'
        )
        if hasattr(self, 'full_control_moveit_pipeline_label'):
            self.full_control_moveit_pipeline_label.setVisible(full_backend_is_moveit)
        if hasattr(self, 'full_control_moveit_pipeline_combo'):
            self.full_control_moveit_pipeline_combo.setVisible(full_backend_is_moveit)
        if hasattr(self, 'full_control_moveit_planner_id_label'):
            self.full_control_moveit_planner_id_label.setVisible(
                full_backend_is_moveit and full_pipeline_is_pilz
            )
        if hasattr(self, 'full_control_moveit_planner_id_combo'):
            self.full_control_moveit_planner_id_combo.setVisible(
                full_backend_is_moveit and full_pipeline_is_pilz
            )

    def _get_active_planner_context(self):
        """Use the currently open planner tab, falling back to the last Arm/Full tab visited."""
        if hasattr(self, 'tabs'):
            current_index = self.tabs.currentIndex()
            if hasattr(self, 'ARM_TAB_INDEX') and current_index == self.ARM_TAB_INDEX:
                self._active_planner_context = 'arm'
            elif hasattr(self, 'FULL_CONTROL_TAB_INDEX') and current_index == self.FULL_CONTROL_TAB_INDEX:
                self._active_planner_context = 'full'

        return getattr(self, '_active_planner_context', 'arm')

    def _update_joint_control_tab_state(self):
        """Enable the Gantry Control tab only when it is usable.

        The gantry exists only in full mode (mode:=full), so it is disabled while a
        base (mode:=base) process runs, and also when moveit is selected in the
        active planner tab.
        """
        planner_context = self._get_active_planner_context()
        planner_backend = self._get_planner_backend(context=planner_context)
        is_moveit_active = (planner_backend == 'moveit')

        base_running = any(self._is_base_tab_state_process(key) for key in self.process_map)

        if hasattr(self, 'tabs') and hasattr(self, 'JOINT_TAB_INDEX'):
            self.tabs.setTabEnabled(
                self.JOINT_TAB_INDEX, not is_moveit_active and not base_running
            )

    def _update_full_control_init_box_state(self):
        """Disable Full Control initialization while simulation mode is selected."""
        full_sim_mode = self.full_control_sim_mode_combo.currentText() if hasattr(self, "full_control_sim_mode_combo") else 'false'
        should_disable = (full_sim_mode == 'true')
        if hasattr(self, "full_control_init_box"):
            self.full_control_init_box.setEnabled(not should_disable)

    def _update_headless_visibility(self):
        """Show simulation-only selectors only when simulation mode is true."""
        base_sim_mode = self.sim_mode_combo.currentText() if hasattr(self, "sim_mode_combo") else 'false'
        base_visible = (base_sim_mode == 'true')
        if hasattr(self, "base_headless_label"):
            self.base_headless_label.setVisible(base_visible)
        if hasattr(self, "base_headless_combo"):
            self.base_headless_combo.setVisible(base_visible)

        full_sim_mode = self.full_control_sim_mode_combo.currentText() if hasattr(self, "full_control_sim_mode_combo") else 'false'
        full_visible = (full_sim_mode == 'true')
        if hasattr(self, "full_control_headless_label"):
            self.full_control_headless_label.setVisible(full_visible)
        if hasattr(self, "full_control_headless_combo"):
            self.full_control_headless_combo.setVisible(full_visible)

    def _set_tab_enabled(self, tab_index, enabled):
        """Enable or disable a tab (make it clickable or unclickable)"""
        self.tabs.setTabEnabled(tab_index, enabled)

    def _update_tab_states_for_base(self):
        """Update tab states based on base control processes"""
        # Any base (non-full) process running?
        base_running = any(self._is_base_tab_state_process(key) for key in self.process_map)

        # Disable Full Control while a base process runs.
        self._set_tab_enabled(self.FULL_CONTROL_TAB_INDEX, not base_running)

        # The gantry only exists in full mode, so the Gantry Control tab is disabled
        # while a base (mode:=base) process runs. _update_joint_control_tab_state()
        # accounts for the running base process (and the moveit constraint).
        self._update_joint_control_tab_state()

    def _update_tab_states_for_full_control(self):
        """Update tab states based on full control processes"""
        full_running = any(
            key in self.process_map
            for key in [
                'full_mobile_manipulator',
                'full_localization',
                'full_nav2',
            ]
        )

        # Disable Base while a full-control process runs (Full + Gantry stay enabled)
        self._set_tab_enabled(self.BASE_TAB_INDEX, not full_running)

    def _spin_ros(self):
        """Safely spin ROS, checking context is valid first"""
        try:
            if rclpy.ok():
                rclpy.spin_once(self.node, timeout_sec=0)
        except Exception:
            pass  # Ignore errors if context is shutting down

    def _on_parking_active(self, msg):
        """Cache the controller's parking-active flag (true while parking is running)."""
        self._parking_active = bool(msg.data)

    def _on_tab_changed(self, index, tabs):
        """Handle tab change between the Base / Gantry / Full Control / FSM tabs."""
        self._update_joint_control_tab_state()

    @staticmethod
    def _strip_html(text):
        """Strip HTML tags to get plain text for filter matching."""
        import re as _re
        return _re.sub(r'<[^>]+>', '', text)

    def _log_append(self, widget, content, add_newline=True):
        """Store entry in the per-widget log buffer and render it if it passes the current filter."""
        plain = self._strip_html(content)
        self.tab_log_entries.setdefault(id(widget), []).append((content, plain, add_newline))
        filt = self.tab_filter_inputs.get(id(widget))
        term = filt.text().strip().lower() if filt else ''
        if not term or term in plain.lower():
            self._append_to_text_widget(widget, content, add_newline)

    def _log_apply_filter(self, widget):
        """Rebuild the widget showing only entries whose plain text matches the current filter."""
        filt = self.tab_filter_inputs.get(id(widget))
        term = filt.text().strip().lower() if filt else ''
        scrollbar = widget.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10
        widget.clear()
        for h, plain, nl in self.tab_log_entries.get(id(widget), []):
            if not term or term in plain.lower():
                self._append_to_text_widget(widget, h, nl)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def _ansi_to_html(self, text):
        """Convert ANSI color codes to HTML"""
        # ANSI color code mapping
        ansi_colors = {
            '30': '#adbac7',  # black (using default text color)
            '31': '#f47067',  # red
            '32': '#57ab5a',  # green
            '33': '#c69026',  # yellow
            '34': '#539bf5',  # blue
            '35': '#b083f0',  # magenta
            '36': '#76e3ea',  # cyan
            '37': '#adbac7',  # white
            '90': '#636e7b',  # bright black (gray)
            '91': '#ff938a',  # bright red
            '92': '#6bc46d',  # bright green
            '93': '#daaa3f',  # bright yellow
            '94': '#6cb6ff',  # bright blue
            '95': '#d2a8ff',  # bright magenta
            '96': '#96d0ff',  # bright cyan
            '97': '#cdd9e5',  # bright white
        }
 
        # First, expand tabs to spaces (8-char tab stops like terminal)
        expanded_text = ''
        col = 0
        for char in text:
            if char == '\t':
                # Calculate spaces needed to reach next 8-char boundary
                spaces_needed = 8 - (col % 8)
                expanded_text += ' ' * spaces_needed
                col += spaces_needed
            elif char == '\033':
                # ANSI escape sequence doesn't affect column position
                expanded_text += char
            else:
                expanded_text += char
                if char not in '\033\r':  # Don't count escape sequences
                    col += 1
 
        # Now process ANSI codes
        result = []
        current_color = None
 
        # Split by ANSI codes
        parts = re.split(r'\033\[([0-9;]+)m', expanded_text)
 
        for i, part in enumerate(parts):
            if i % 2 == 0:  # Text content
                if part:
                    # Escape HTML special chars
                    escaped_part = part.replace('&', '&amp;').replace('<', '&lt;').replace('>', '&gt;')
                    if current_color:
                        result.append(f"<span style='color: {current_color};'>{escaped_part}</span>")
                    else:
                        result.append(escaped_part)
            else:  # ANSI code
                codes = part.split(';')
                if '0' in codes or not codes[0]:  # Reset
                    current_color = None
                else:
                    # Look for color code
                    for code in codes:
                        if code in ansi_colors:
                            current_color = ansi_colors[code]
                            break
 
        return ''.join(result)
    
    def _append_to_text_widget(self, text_widget, html_content, add_newline=True):
        """
        Append content to a text widget with smart scrolling.
        Only auto-scrolls if the user is already viewing the bottom.
        
        Args:
            text_widget: QTextEdit widget to append to
            html_content: HTML content to append
            add_newline: Whether to add a newline after the content
        """
        # Check if scrollbar is at the bottom before appending
        scrollbar = text_widget.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10  # Small tolerance for rounding
        
        # Save the current scroll position
        old_scroll_value = scrollbar.value()
        
        # Append the content
        cursor = text_widget.textCursor()
        cursor.movePosition(cursor.End)
        cursor.insertHtml(html_content)
        if add_newline:
            cursor.insertText('\n')
        
        # Restore scroll position or scroll to bottom as appropriate
        if was_at_bottom:
            # User was at bottom, scroll to new bottom
            scrollbar.setValue(scrollbar.maximum())
        else:
            # User was scrolled up, maintain their position
            scrollbar.setValue(old_scroll_value)

    def _handle_freedrive_output(self, process, status_text):
        """Stream freedrive command output while suppressing repetitive keepalive spam."""
        output = process.readAllStandardOutput().data().decode()
        if not output:
            return

        for line in output.splitlines():
            stripped = line.strip()
            if not stripped:
                continue
            if stripped == 'publisher: beginning loop' or stripped.startswith('publishing #'):
                continue

            self._append_to_text_widget(status_text, self._ansi_to_html(line))

    def _build_freedrive_command_text(self, ros2_args):
        """Format a ros2 command for status display."""
        return 'ros2 ' + ' '.join(shlex.quote(arg) for arg in ros2_args)

    def _run_freedrive_command(self, process_key, ros2_args, status_text, finished_callback):
        """Run a one-shot freedrive-related ros2 command."""
        if process_key in self.process_map:
            existing_process = self.process_map[process_key]
            if existing_process.state() == QProcess.Running:
                existing_process.kill()
                existing_process.waitForFinished(1000)
            del self.process_map[process_key]

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self._handle_freedrive_output(process, status_text)
        )
        process.finished.connect(
            lambda exit_code, _exit_status: self._on_freedrive_command_finished(
                process_key, exit_code, status_text, finished_callback
            )
        )

        cmd_str = self._build_freedrive_command_text(ros2_args)
        self._append_to_text_widget(status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        process.start('ros2', ros2_args)
        self.process_map[process_key] = process

    def _on_freedrive_command_finished(self, process_key, exit_code, status_text, finished_callback):
        """Clean up a one-shot freedrive command and continue the sequence."""
        if process_key in self.process_map:
            del self.process_map[process_key]
        finished_callback(exit_code, status_text)

    def _interrupt_freedrive_process(self, process_key, status_text, label):
        """Best-effort Ctrl+C style stop for a freedrive-related process."""
        if process_key not in self.process_map:
            return False

        process = self.process_map[process_key]
        try:
            process.finished.disconnect()
        except Exception:
            pass

        pid = process.processId()
        if pid:
            try:
                os.kill(pid, signal.SIGINT)
                status_text.append(f"⏹ Sent Ctrl+C to {label}")
            except ProcessLookupError:
                pass
            except Exception as exc:
                status_text.append(
                    f"<span style='color: #c69026;'>⚠ Could not send Ctrl+C to {label}: {exc}</span>"
                )

        if process.state() == QProcess.Running:
            process.waitForFinished(1500)
        if process.state() == QProcess.Running:
            process.terminate()
            process.waitForFinished(1000)
        if process.state() == QProcess.Running:
            process.kill()
            process.waitForFinished(1000)

        if process_key in self.process_map:
            del self.process_map[process_key]
        return True

    def _stop_freedrive_start_processes(self, status_text):
        """Interrupt the freedrive start-side processes if they are still running."""
        self._interrupt_freedrive_process(
            'freedrive_start_switch',
            status_text,
            'freedrive controller switch',
        )
        self._interrupt_freedrive_process(
            'freedrive_enable_publisher',
            status_text,
            'freedrive keepalive publisher',
        )

    def _stop_freedrive_stop_processes(self, status_text):
        """Interrupt any lingering freedrive stop-side commands before restarting."""
        self._interrupt_freedrive_process(
            'freedrive_stop_disable',
            status_text,
            'freedrive disable publisher',
        )
        self._interrupt_freedrive_process(
            'freedrive_stop_switch',
            status_text,
            'trajectory controller restore switch',
        )

    def _freedrive_namespace_prefix(self):
        """Namespace prefix derived from the active controller_manager (e.g. '/arm' or '')."""
        cm = self.freedrive_controller_manager or '/controller_manager'
        suffix = '/controller_manager'
        return cm[:-len(suffix)] if cm.endswith(suffix) else ''

    def _freedrive_enable_topic(self):
        """Enable/keepalive topic for the freedrive controller, namespaced to its controller_manager."""
        return f"{self._freedrive_namespace_prefix()}/freedrive_mode_controller/enable_freedrive_mode"

    def _start_freedrive_publisher(self, status_text):
        """Start the persistent freedrive keepalive publisher."""
        process_key = 'freedrive_enable_publisher'
        ros2_args = [
            'topic',
            'pub',
            '--rate',
            '2',
            self._freedrive_enable_topic(),
            'std_msgs/msg/Bool',
            '{data: true}',
        ]

        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(
            lambda: self._handle_freedrive_output(process, status_text)
        )
        process.finished.connect(
            lambda exit_code, _exit_status: self._on_freedrive_publisher_finished(
                exit_code, status_text
            )
        )

        cmd_str = self._build_freedrive_command_text(ros2_args)
        self._append_to_text_widget(status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        process.start('ros2', ros2_args)
        self.process_map[process_key] = process

    def _on_freedrive_publisher_finished(self, exit_code, status_text):
        """Handle the persistent freedrive publisher exiting unexpectedly."""
        process_key = 'freedrive_enable_publisher'
        if process_key in self.process_map:
            del self.process_map[process_key]

        self.freedrive_active = False
        self._update_freedrive_button_ui()

        if exit_code == 0:
            status_text.append("⏹ Freedrive keepalive publisher stopped")
        else:
            status_text.append(
                f"<span style='color: #f47067;'>✗ Freedrive keepalive publisher exited unexpectedly (exit code {exit_code})</span>"
            )

    def _update_freedrive_button_ui(self):
        """Keep freedrive buttons in sync across Arm Control and Full Control tabs."""
        if self.freedrive_active:
            text = "Stop Freedrive"
            style = "background-color: #4CAF50; color: white; font-weight: bold;"
        else:
            text = "Start Freedrive"
            style = ""

        if hasattr(self, "btn_arm_freedrive"):
            self.btn_arm_freedrive.setText(text)
            self.btn_arm_freedrive.setStyleSheet(style)
            self.btn_arm_freedrive.setEnabled(True)

        if hasattr(self, "btn_full_control_freedrive"):
            self.btn_full_control_freedrive.setText(text)
            self.btn_full_control_freedrive.setStyleSheet(style)
            self.btn_full_control_freedrive.setEnabled(True)

    def _detect_freedrive_controller_manager(self):
        """Find the controller_manager node that hosts freedrive_mode_controller.

        The arm stack can run under the /arm namespace (e.g. the MoveIt backend),
        in which case its controllers live on /arm/controller_manager rather than
        the root /controller_manager. Falls back to /controller_manager.
        """
        try:
            nodes = subprocess.run(
                ['bash', '-lc', "ros2 node list 2>/dev/null | grep -E '(^|/)controller_manager$' | sort -u"],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except Exception:
            return '/controller_manager'

        candidates = [line.strip() for line in nodes.stdout.splitlines() if line.strip()]
        for cm in candidates:
            try:
                result = subprocess.run(
                    ['bash', '-lc', f"ros2 control list_controllers -c {cm} 2>/dev/null"],
                    capture_output=True,
                    text=True,
                    timeout=15,
                )
            except Exception:
                continue
            if 'freedrive_mode_controller' in result.stdout:
                return cm

        return '/controller_manager'

    def _get_freedrive_controller_check_command(self, controller_manager='/controller_manager'):
        """Return the shell command used to detect freedrive-related controllers."""
        return f"ros2 control list_controllers -c {controller_manager} | grep -E 'freedrive|trajectory_controller'"

    def _get_controller_states(self, status_text=None):
        """Return freedrive-related controller states parsed from the direct shell command output."""
        controller_manager = self._detect_freedrive_controller_manager()
        self.freedrive_controller_manager = controller_manager
        check_command = self._get_freedrive_controller_check_command(controller_manager)
        try:
            result = subprocess.run(
                ['bash', '-lc', check_command],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except subprocess.TimeoutExpired:
            if status_text is not None:
                self._append_to_text_widget(
                    status_text,
                    "<span style='color: #c69026;'>Freedrive controller check timed out</span>",
                )
            return {}
        except Exception as exc:
            if status_text is not None:
                self._append_to_text_widget(
                    status_text,
                    f"<span style='color: #c69026;'>Freedrive controller check failed: {html.escape(str(exc))}</span>",
                )
            return {}

        controller_output = f"{result.stdout}\n{result.stderr or ''}"
        controller_states = {}

        if status_text is not None:
            cmd_str = check_command
            self._append_to_text_widget(status_text, f"<b style='color: #57ab5a;'>▶ {html.escape(cmd_str)}</b>")
            stripped_output = controller_output.strip()
            if stripped_output:
                self._append_to_text_widget(
                    status_text,
                    (
                        "<pre style=\"margin: 0; font-family: 'Courier New', monospace;\">"
                        f"{html.escape(stripped_output)}"
                        "</pre>"
                    ),
                    add_newline=False,
                )

        for raw_line in controller_output.splitlines():
            line = re.sub(r'\x1b\[[0-9;]*m', '', raw_line).strip()
            if not line:
                continue
            if line.startswith('[INFO]') or line.startswith('[WARN]') or line.startswith('[ERROR]'):
                continue

            match = re.match(
                r'^([A-Za-z0-9_./-]+)\s+([A-Za-z0-9_./:-]+)\s+(active|inactive|unconfigured|finalized)\b',
                line,
            )
            if not match:
                continue

            controller_states[match.group(1)] = match.group(3)

        return controller_states

    def _get_available_trajectory_controller(self, controller_states=None):
        """Return the preferred trajectory controller if one is loaded."""
        if controller_states is None:
            controller_states = self._get_controller_states()
        trajectory_controller_candidates = (
            'passthrough_trajectory_controller',
            'joint_trajectory_controller',
            'scaled_joint_trajectory_controller',
        )

        for controller_name in trajectory_controller_candidates:
            if controller_states.get(controller_name) == 'active':
                return controller_name

        for controller_name in trajectory_controller_candidates:
            if controller_name in controller_states:
                return controller_name

        return None

    def _freedrive_controllers_are_loaded(self, status_text=None):
        """Return True when freedrive and a compatible trajectory controller are available."""
        controller_states = self._get_controller_states(status_text=status_text)
        trajectory_controller = self._get_available_trajectory_controller(controller_states)
        self.freedrive_trajectory_controller = trajectory_controller
        return bool(trajectory_controller and 'freedrive_mode_controller' in controller_states)

    def _show_no_controllers_running_message(self, status_text):
        """Notify the user that freedrive cannot start because controllers are unavailable."""
        message = (
            "Freedrive requires freedrive_mode_controller and either "
            "joint_trajectory_controller or scaled_joint_trajectory_controller."
        )
        QMessageBox.warning(self, 'Freedrive Mode', message)
        status_text.append(f"<span style='color: #c69026;'>{message}</span>")

    def _confirm_freedrive_start(self):
        """Ask the user to confirm the freedrive start sequence."""
        reply = QMessageBox.question(
            self,
            'Confirm Freedrive',
            "Are you sure you want to start freedrive?\n\n"
            "Make sure you have calibrated the PAYLOAD in the teach pendant. "
            "If not hold the arm by hand before it falls or moves up.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )
        return reply == QMessageBox.Yes

    def toggle_freedrive(self, context='arm'):
        """Toggle freedrive mode with confirmation before activation."""
        status_text = self._get_status_text_for_context(context)

        if self.freedrive_active:
            self._stop_freedrive(status_text)
            return

        if not self._freedrive_controllers_are_loaded(status_text=status_text):
            self._show_no_controllers_running_message(status_text)
            return

        if not self._confirm_freedrive_start():
            return

        self._start_freedrive(status_text)

    def _start_freedrive(self, status_text):
        """Activate the freedrive controller and start the enable publisher."""
        trajectory_controller = self.freedrive_trajectory_controller or self._get_available_trajectory_controller()
        if not trajectory_controller:
            self.freedrive_active = False
            self._update_freedrive_button_ui()
            self._show_no_controllers_running_message(status_text)
            return

        self.freedrive_trajectory_controller = trajectory_controller
        self._stop_freedrive_stop_processes(status_text)
        self._stop_freedrive_start_processes(status_text)
        self.freedrive_active = True
        self.freedrive_transition_in_progress = False
        self.freedrive_transition_target_active = False
        self._update_freedrive_button_ui()
        status_text.append("<span style='color: #57ab5a; font-weight: bold;'>✓ Freedrive mode requested</span>")
        self._run_freedrive_command(
            'freedrive_start_switch',
            [
                'control',
                'switch_controllers',
                '-c',
                self.freedrive_controller_manager,
                '--deactivate',
                trajectory_controller,
                'force_mode_controller',
                '--activate',
                'freedrive_mode_controller',
            ],
            status_text,
            self._on_freedrive_start_switch_finished,
        )

    def _on_freedrive_start_switch_finished(self, exit_code, status_text):
        """Start the keepalive publisher after the controller switch succeeds."""
        if exit_code == 0:
            status_text.append("✓ Freedrive controller activated")
            self._start_freedrive_publisher(status_text)
        else:
            self.freedrive_active = False
            self._update_freedrive_button_ui()
            status_text.append(
                f"<span style='color: #f47067;'>✗ Freedrive controller switch failed (exit code {exit_code})</span>"
            )

    def _stop_freedrive(self, status_text):
        """Stop the freedrive publisher, disable freedrive, and restore trajectory control."""
        trajectory_controller = self.freedrive_trajectory_controller or self._get_available_trajectory_controller()
        self.freedrive_active = False
        self.freedrive_transition_in_progress = False
        self.freedrive_transition_target_active = False
        self._update_freedrive_button_ui()
        self._stop_freedrive_start_processes(status_text)
        self._stop_freedrive_stop_processes(status_text)
        self._run_freedrive_command(
            'freedrive_stop_disable',
            [
                'topic',
                'pub',
                '--once',
                self._freedrive_enable_topic(),
                'std_msgs/msg/Bool',
                '{data: false}',
            ],
            status_text,
            self._on_freedrive_disable_finished,
        )

    def _on_freedrive_disable_finished(self, exit_code, status_text):
        """Switch controllers back after sending the freedrive disable message."""
        trajectory_controller = self.freedrive_trajectory_controller or self._get_available_trajectory_controller()
        if exit_code == 0:
            status_text.append("✓ Freedrive disable message sent")
        else:
            status_text.append(
                f"<span style='color: #c69026;'>⚠ Freedrive disable message failed (exit code {exit_code}); switching controllers back anyway</span>"
            )

        if not trajectory_controller:
            status_text.append(
                "<span style='color: #f47067;'>✗ Could not determine which trajectory controller to restore</span>"
            )
            return

        self._run_freedrive_command(
            'freedrive_stop_switch',
            [
                'control',
                'switch_controllers',
                '-c',
                self.freedrive_controller_manager,
                '--deactivate',
                'freedrive_mode_controller',
                '--activate',
                trajectory_controller,
                'force_mode_controller',
            ],
            status_text,
            self._on_freedrive_stop_switch_finished,
        )

    def _on_freedrive_stop_switch_finished(self, exit_code, status_text):
        """Finalize freedrive shutdown and restore the button state."""
        if exit_code == 0:
            status_text.append(
                "<span style='color: #57ab5a; font-weight: bold;'>✓ Freedrive mode disabled</span>"
            )
        else:
            status_text.append(
                f"<span style='color: #f47067;'>✗ Failed to restore joint_trajectory_controller (exit code {exit_code})</span>"
            )

    def toggle_base_control_launch(self):
        """Toggle the base-only bringup from the Base Control tab."""
        sim_mode = self.sim_mode_combo.currentText()
        controller_type = self.controller_type_combo.currentText()
        headless = self.base_headless_combo.currentText()

        launch_args = [
            'launch', 'robo_drill', 'platform.launch.py',
            f'sim:={sim_mode}',
            'mode:=base',
            f'controller_type:={controller_type}',
            'odom_tf_from_controller:=true',
            'publish_controller_odom_tf:=true',
            'launch_rviz:=true',
            f'headless:={headless}',
        ]
        if 'mobile_platform' not in self.process_map:
            self.btn_launch_base_robot.setProperty('uses_gazebo', sim_mode == 'true')

        self._toggle_base_process(
            'mobile_platform',
            self.btn_launch_base_robot,
            'Base Robot',
            'ros2',
            launch_args,
        )
        self._update_tab_states_for_base()

    def toggle_full_control_launch(self):
        """Toggle the full robo_drill mobile manipulator (base + gantry) from the Full Control tab."""
        sim_mode = self.full_control_sim_mode_combo.currentText()
        controller_type = self.full_control_controller_type_combo.currentText()
        headless = self.full_control_headless_combo.currentText()

        launch_args = [
            'launch', 'robo_drill', self._get_full_control_launch_file(),
            f'sim:={sim_mode}',
            'mode:=full',
            f'controller_type:={controller_type}',
            'publish_controller_odom_tf:=true',
            'launch_rviz:=true',
            f'headless:={headless}',
        ]
        if 'full_mobile_manipulator' not in self.process_map:
            self.btn_full_control_launch.setProperty('uses_gazebo', sim_mode == 'true')

        self._toggle_base_process(
            'full_mobile_manipulator',
            self.btn_full_control_launch,
            'Full Robot',
            'ros2',
            launch_args,
        )
        self._update_tab_states_for_full_control()

    def toggle_localization(self, mode='base', button=None, sim_combo=None, controller_type_combo=None, headless_combo=None):
        """Toggle localization with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_localization
        if sim_combo is None:
            sim_combo = self.sim_mode_combo
        if controller_type_combo is None:
            controller_type_combo = self.controller_type_combo
        if headless_combo is None:
            headless_combo = self.base_headless_combo if mode == 'base' else self.full_control_headless_combo

        sim_mode = sim_combo.currentText()
        controller_type = controller_type_combo.currentText()
        headless = headless_combo.currentText() if headless_combo else 'false'

        process_key = f'{mode}_localization' if mode != 'base' else 'localization'
        display_name = 'Localization'

        localization_args = [
            'launch', 'robo_drill', 'move_robot.launch.py',
            f'sim:={sim_mode}', f'mode:={mode}', f'controller_type:={controller_type}',
        ]
        if mode == 'full':
            localization_args.append(f'planner_backend:={self._get_planner_backend(context="full")}')
            localization_args.extend(self._get_moveit_launch_args(context='full'))
        localization_args.append(f'headless:={headless}')

        self._toggle_base_process(process_key, button, display_name, 'ros2', localization_args)
        if mode == 'full':
            self._update_tab_states_for_full_control()
        
        # Disable/enable conflicting buttons based on localization state
        disable_buttons = self._get_buttons_to_disable_for_localization(mode)
        for btn in disable_buttons:
            btn.setEnabled(not (process_key in self.process_map))

        # Update tab states only for base mode
        if mode == 'base':
            self._update_tab_states_for_base()

 
    def toggle_view_map(self, mode='base', button=None):
        if button is None:
            button = self.btn_view_map

        # Get package path for rtabmap.db
        try:
            pkg_share = get_package_share_directory('robo_drill')
            db_path = os.path.join(pkg_share, 'maps', 'rtabmap.db')
        except Exception as e:
            status_text = self.full_control_status_text if mode == 'full' else self.base_status_text
            self._log_append(status_text, f"Error: Could not find robo_drill package: {e}")
            return
        
        process_key = 'full_view_map' if mode == 'full' else 'view_map'
        self._toggle_base_process(process_key, button, 'View map',
                                 'rtabmap-databaseViewer', [db_path])
    
    def toggle_nav2(self, mode='base', button=None, sim_combo=None, controller_type_combo=None):
        """Toggle Nav2 with configurable mode parameter"""
        if button is None:
            button = self.btn_launch_nav2
        if sim_combo is None:
            sim_combo = self.sim_mode_combo
        if controller_type_combo is None:
            controller_type_combo = self.controller_type_combo
        
        sim_mode = sim_combo.currentText()
        controller_type = controller_type_combo.currentText()
        process_key = f'{mode}_nav2' if mode != 'base' else 'nav2'
        display_name = 'Nav2'
        
        self._toggle_base_process(process_key, button, display_name,
                                'ros2', ['launch', 'robo_drill', 'navigation_launch.py',
                                        f'use_sim_time:={sim_mode}', f'controller_type:={controller_type}'])
        if mode == 'full':
            self._update_tab_states_for_full_control()
        else:
            self._update_tab_states_for_base()
            
    # Helper methods to get the correct buttons for each mode
    def _get_buttons_to_disable_for_localization(self, mode):
        if mode == 'base':
            return [self.btn_launch_nav2]
        elif mode == 'full':
            return [self.btn_full_control_nav2]
        return []
    
    def _get_localization_button_for_mode(self, mode):
        """Get the localization button for a given mode"""
        if mode == 'base':
            return self.btn_launch_localization
        elif mode == 'full':
            return self.btn_full_control_localization
        return None

    def clear_full_control_status(self):
        widget = self.full_control_status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_full_control_status.setVisible(True)

    def restore_full_control_status(self):
        widget = self.full_control_status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_full_control_status.setVisible(False)

    def toggle_rqt(self, mode='base', button=None):
        """Toggle RQT on/off"""
        if button is None:
            button = self.btn_launch_rqt
        process_key = 'full_rqt' if mode == 'full' else 'rqt'
        self._toggle_base_process(process_key, button, 'RQT', 'rqt', [])
 
    def run_ps_ros(self):
        """Run ps aux | grep ros2 command"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = 'ps aux | grep -E \'ros2|robot\' | grep -v grep'
        self._log_append(self.base_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command using shell to support pipe
        process_key = 'ps_ros2'
        process.finished.connect(lambda: self._cleanup_ps_ros(process_key))
        process.start('bash', ['-c', 'ps aux | grep -E \'ros2|robot\' | grep -v grep'])
        self.process_map[process_key] = process
 
    def _cleanup_ps_ros(self, process_key):
        """Clean up finished ps aux process"""
        if process_key in self.process_map:
            del self.process_map[process_key]

    def _build_list_all_controllers_script(self):
        """Shell script that lists controllers for every detected controller_manager node."""
        return (
            "manager_nodes=$(ros2 node list 2>/dev/null | grep -E '(^|/)controller_manager$' | sort -u); "
            "if [ -z \"$manager_nodes\" ]; then "
            "  echo \"No controller_manager nodes found.\"; "
            "  exit 1; "
            "fi; "
            "for cm in $manager_nodes; do "
            "  echo \"===== $cm =====\"; "
            "  timeout 8 ros2 control list_controllers -c \"$cm\" || echo \"[WARN] Failed to query $cm\"; "
            "  echo; "
            "done"
        )
 
    def run_list_base_controllers(self):
        """List controllers for all detected controller_manager nodes."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))
 
        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.base_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command with timeout (10 seconds)
        process_key = 'list_base_controllers'
        process.finished.connect(lambda: self._cleanup_list_base_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process
 
    def _cleanup_list_base_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.base_status_text, f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.base_status_text, "✓ List controllers command completed")

    def run_list_full_controllers(self):
        """List controllers for all detected controller_manager nodes (Full Control tab)."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))

        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command with timeout (10 seconds)
        process_key = 'full_list_base_controllers'
        process.finished.connect(lambda: self._cleanup_list_full_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process

    def _cleanup_list_full_controllers(self, process_key):
        """Clean up finished list controllers process (Full Control tab)"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.full_control_status_text,
                    f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>"
                )
            else:
                self._log_append(self.full_control_status_text, "✓ List controllers command completed")
 
    def refresh_topics_list(self):
        """Refresh the list of available ROS2 topics"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Display command in status
        cmd_str = 'ros2 topic list'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Start process and capture output
        process.finished.connect(lambda: self._on_topics_list_finished(process))
        process.start('ros2', ['topic', 'list'])
        
    def _on_topics_list_finished(self, process):
        """Process the output of ros2 topic list command"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        
        # Clear and populate the combobox
        self.topics_combo.clear()
        topics = [line.strip() for line in output.split('\n') if line.strip()]
        
        if topics:
            self.topics_combo.addItems(topics)
            self._log_append(self.full_control_status_text, f"✓ Found {len(topics)} topics")
        else:
            self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ No topics found</span>")
    
    def check_topic_bandwidth(self):
        """Check bandwidth of selected topic"""
        topic = self.topics_combo.currentText()
        if not topic:
            self.topic_info_display.setPlainText("⚠ No topic selected")
            return
        
        self.topic_info_display.setPlainText("Measuring bandwidth...")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Store flags and buffers for this process
        self._bw_output_buffer = ""
        self._bw_result_found = False
        process.readyReadStandardOutput.connect(lambda: self._accumulate_bw_output(process))
        
        # Start process with timeout
        process_key = 'topic_bw'
        process.finished.connect(lambda: self._on_bandwidth_finished(process))
        process.start('timeout', ['4', 'ros2', 'topic', 'bw', topic])
        self.process_map[process_key] = process
    
    def _accumulate_bw_output(self, process):
        """Accumulate bandwidth output as it arrives and parse first result"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        self._bw_output_buffer += output
        
        # If we haven't found a result yet, try to parse it now
        if not self._bw_result_found:
            lines = self._bw_output_buffer.split('\n')
            for line in lines:
                if 'Message size mean:' in line:
                    try:
                        parts = line.split('Message size mean:')
                        if len(parts) > 1:
                            mean_part = parts[1].strip().split()[0:2]  # Get "0.07 MB"
                            msg_size_mean = ' '.join(mean_part)
                            self.topic_info_display.setPlainText(f"Message size mean: {msg_size_mean}")
                            self._bw_result_found = True
                            # Kill the process since we got what we need
                            if 'topic_bw' in self.process_map:
                                self.process_map['topic_bw'].kill()
                            break
                    except:
                        pass
    
    def check_topic_frequency(self):
        """Check frequency of selected topic"""
        topic = self.topics_combo.currentText()
        if not topic:
            self.topic_info_display.setPlainText("⚠ No topic selected")
            return
        
        self.topic_info_display.setPlainText("Measuring frequency...")
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Store flags and buffers for this process
        self._hz_output_buffer = ""
        self._hz_result_found = False
        process.readyReadStandardOutput.connect(lambda: self._accumulate_hz_output(process))
        
        # Start process with timeout
        process_key = 'topic_hz'
        process.finished.connect(lambda: self._on_frequency_finished(process))
        process.start('timeout', ['4', 'ros2', 'topic', 'hz', topic])
        self.process_map[process_key] = process
    
    def _accumulate_hz_output(self, process):
        """Accumulate frequency output as it arrives and parse first result"""
        output = process.readAllStandardOutput().data().decode('utf-8')
        self._hz_output_buffer += output
        
        # If we haven't found a result yet, try to parse it now
        if not self._hz_result_found:
            lines = self._hz_output_buffer.split('\n')
            for line in lines:
                if 'average rate:' in line:
                    try:
                        parts = line.split('average rate:')
                        if len(parts) > 1:
                            avg_rate = parts[1].strip().split()[0]
                            self.topic_info_display.setPlainText(f"Average rate: {avg_rate} Hz")
                            self._hz_result_found = True
                            # Kill the process since we got what we need
                            if 'topic_hz' in self.process_map:
                                self.process_map['topic_hz'].kill()
                            break
                    except:
                        pass
    
    def echo_topic_once(self):
        """Echo selected topic once"""
        topic = self.topics_combo.currentText()
        if not topic:
            self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ No topic selected</span>")
            return
        
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
        
        # Display command
        cmd_str = f'timeout 10 ros2 topic echo --once {topic}'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Start process with timeout
        process_key = f'topic_echo_{topic}'
        process.finished.connect(lambda: self._cleanup_topic_command(process_key))
        process.start('timeout', ['10', 'ros2', 'topic', 'echo', '--once', topic])
        self.process_map[process_key] = process
    
    def _on_bandwidth_finished(self, process):
        """Handle bandwidth process completion"""
        if 'topic_bw' in self.process_map:
            del self.process_map['topic_bw']
        
        # If we didn't get a result during execution, try one more time
        if not self._bw_result_found:
            final_output = process.readAllStandardOutput().data().decode('utf-8')
            output = self._bw_output_buffer + final_output
            
            if not output.strip():
                self.topic_info_display.setPlainText("⚠ No data received")
                return
            
            # Try to find the result in the complete output
            lines = output.split('\n')
            for line in lines:
                if 'Message size mean:' in line:
                    try:
                        parts = line.split('Message size mean:')
                        if len(parts) > 1:
                            mean_part = parts[1].strip().split()[0:2]
                            msg_size_mean = ' '.join(mean_part)
                            self.topic_info_display.setPlainText(f"Message size mean: {msg_size_mean}")
                            return
                    except:
                        pass
            
            self.topic_info_display.setPlainText("⚠ Could not parse bandwidth data")
        
        # Clear buffer
        self._bw_output_buffer = ""
    
    def _on_frequency_finished(self, process):
        """Handle frequency process completion"""
        if 'topic_hz' in self.process_map:
            del self.process_map['topic_hz']
        
        # If we didn't get a result during execution, try one more time
        if not self._hz_result_found:
            final_output = process.readAllStandardOutput().data().decode('utf-8')
            output = self._hz_output_buffer + final_output
            
            if not output.strip():
                self.topic_info_display.setPlainText("⚠ No data received")
                return
            
            # Try to find the result in the complete output
            lines = output.split('\n')
            for line in lines:
                if 'average rate:' in line:
                    try:
                        parts = line.split('average rate:')
                        if len(parts) > 1:
                            avg_rate = parts[1].strip().split()[0]
                            self.topic_info_display.setPlainText(f"Average rate: {avg_rate} Hz")
                            return
                    except:
                        pass
            
            self.topic_info_display.setPlainText("⚠ Could not parse frequency data")
        
        # Clear buffer
        self._hz_output_buffer = ""
    
    def _cleanup_topic_command(self, process_key):
        """Clean up finished topic command process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code == 124:  # timeout exit code
                self._log_append(self.full_control_status_text, "<span style='color: #c69026;'>⚠ Command timed out</span>")
            elif exit_code != 0:
                self._log_append(self.full_control_status_text, f"<span style='color: #c69026;'>⚠ Command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.full_control_status_text, "✓ Command completed")

    def _toggle_process(self, process_key, button, name, program, args):
        """Toggle a process on/off and update button state"""
        if process_key in self.process_map:
            # Stop the process
            process = self.process_map[process_key]
            # Disconnect finished signal to prevent race condition
            try:
                process.finished.disconnect()
            except:
                pass
 
            # For launch processes, kill only child processes (including rviz2)
            if process_key in ['ur_control', 'arm_launch']:
                pid = process.processId()
                if pid:
                    try:
                        # Kill all children of this specific launch process
                        subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                    except:
                        pass
                # Extra cleanup for child ROS processes spawned by launch files
                self._cleanup_ros_children_of_pid(pid)
                # Kill Gazebo processes when stopping Arm launch
                if process_key == 'arm_launch':
                    self._kill_gazebo_processes()
 
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
 
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.status_text, f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
            process.finished.connect(lambda: self._on_process_finished(process_key, button, name))

            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(self.status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
 
    def _park_chassis_before_stop(self, process_key, status_text):
        """Call the chassis-parking service and wait for alignment before stopping the
        base-robot bringup, so the launch shuts down with the chassis aligned to the
        turret. This is the safe, deterministic alternative to parking on Ctrl+C: the
        maneuver runs while the control loop is alive and nothing is tearing it down.

        Best-effort: if the service is unavailable (e.g. diff controller, or the
        controller is not up) it skips quickly and lets normal shutdown proceed.
        """
        # Bringups that run the base controller and should park the chassis before
        # shutdown. Base Control tab: Launch Base Robot, Start Localization. Full
        # Control tab: Launch Full Robot, Start Localization. (The full-robot base
        # controller is also /sim_controller; if it were ever named differently, the
        # service-list check below skips gracefully.)
        if process_key not in (
            'mobile_platform', 'localization',
            'full_mobile_manipulator', 'full_localization',
        ):
            return

        service = '/sim_controller/park_now'

        # Fast availability check so diff-mode / no-controller stops don't block on a
        # service call that would otherwise wait for a service that never appears.
        try:
            listed = subprocess.run(
                ['ros2', 'service', 'list'],
                capture_output=True, text=True, timeout=5)
            if service not in (listed.stdout or ''):
                return  # not present (e.g. diff controller) -> skip silently
        except Exception:
            return

        self._log_append(status_text, "🅿 Parking chassis before shutdown...")

        try:
            result = subprocess.run(
                ['ros2', 'service', 'call', service, 'std_srvs/srv/Trigger'],
                capture_output=True, text=True, timeout=8)
        except Exception as e:
            self._log_append(status_text, f"⚠ Park service call failed ({e}); proceeding to shutdown.")
            return
        if result.returncode != 0 or 'success=True' not in (result.stdout or ''):
            self._log_append(status_text, "⚠ Park service call did not succeed; proceeding to shutdown.")
            return

        # Wait on the controller's /sim_controller/parking_active flag: it is true while
        # the maneuver runs and false when the chassis is aligned. We keep waiting for as
        # long as the flag stays active (so large-angle parks, which can take ~30 s near a
        # half turn, are never cut short), and finish the instant it goes back to false.
        # If it never goes active within a short grace (robot already aligned, nothing to
        # do), we proceed. A generous absolute ceiling guards against a hung controller so
        # the UI can never block indefinitely.
        absolute_deadline = time.time() + 120.0
        first_active_grace = time.time() + 3.0
        saw_active = False
        while time.time() < absolute_deadline:
            try:
                if rclpy.ok():
                    rclpy.spin_once(self.node, timeout_sec=0)
            except Exception:
                pass
            QApplication.processEvents()
            if self._parking_active:
                saw_active = True
            elif saw_active:
                # active -> inactive transition: parking finished.
                self._log_append(status_text, "✓ Chassis aligned; proceeding to shutdown.")
                return
            elif time.time() > first_active_grace:
                # never went active: already aligned / nothing to park.
                self._log_append(status_text, "✓ Chassis already aligned; proceeding to shutdown.")
                return
            time.sleep(0.05)
        self._log_append(status_text, "⚠ Parking did not confirm in time; proceeding to shutdown.")

    def _toggle_base_process(self, process_key, button, name, program, args):
        # Decide status widget
        status_text = self.full_control_status_text if process_key.startswith('full') else self.base_status_text

        if process_key in self.process_map:
            # ===== STOP PROCESS =====
            process = self.process_map[process_key]

            # Align the chassis with the turret before shutting down the base robot, so
            # the launch stops with the robot parked. Best-effort and bounded.
            self._park_chassis_before_stop(process_key, status_text)

            # Disconnect finished to avoid double cleanup
            try:
                process.finished.disconnect()
            except Exception:
                pass

            pid = process.processId()

            # Try graceful SIGINT (Ctrl+C equivalent)
            if pid:
                try:
                    os.kill(pid, signal.SIGINT)
                    process.waitForFinished(3000)
                except ProcessLookupError:
                    # Process already gone
                    pass
                except Exception as e:
                    self._log_append(status_text, f"⚠ Could not send SIGINT: {e}")

            # If still running, escalate
            if process.state() == QProcess.Running:
                process.terminate()
                process.waitForFinished(2000)
            if process.state() == QProcess.Running:
                process.kill()
                process.waitForFinished(2000)

            # Kill orphaned launch children/Gazebo for simulator-backed launches.
            if self._is_robot_bringup_process(process_key) and pid:
                try:
                    subprocess.run(['pkill', '-9', '-P', str(pid)], timeout=2, stderr=subprocess.DEVNULL)
                except Exception:
                    pass
                self._cleanup_ros_children_of_pid(pid)

            uses_gazebo = bool(button.property('uses_gazebo'))

            # Kill Gazebo processes when stopping localization or a sim-backed full robot bringup
            if (
                'localization' in process_key
                or (self._is_robot_bringup_process(process_key) and uses_gazebo)
            ):
                self._kill_gazebo_processes()

            # Final cleanup
            if process_key in self.process_map:
                del self.process_map[process_key]

            button.setText(self._get_base_process_start_text(process_key, name))
            button.setStyleSheet("")
            if self._is_robot_bringup_process(process_key):
                button.setProperty('uses_gazebo', False)
            self._log_append(status_text, f"⏹ Stopped {name}")

            # Re‑enable buttons disabled while localization was running
            if 'localization' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                for btn in self._get_buttons_to_disable_for_localization(mode):
                    btn.setEnabled(True)

            # Only base mode affects tab states
            if not process_key.startswith('full') and self._is_base_tab_state_process(process_key):
                self._update_tab_states_for_base()

        else:
            # ===== START PROCESS (unchanged) =====
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)

            if process_key.startswith('full'):
                process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
            else:
                process.readyReadStandardOutput.connect(lambda: self.handle_base_output(process))

            process.finished.connect(lambda: self._on_base_process_finished(process_key, button, name))

            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(status_text, f"<b style='color:#57ab5a;'>▶ {cmd_str}</b>")

            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(self._get_base_process_stop_text(process_key, name))
            button.setStyleSheet("background-color:#4CAF50; color:white; font-weight:bold;")


            
    def handle_full_control_output(self, process):
        """Handle output for full control processes (outputs to full_control_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages
                if 'process has died' in line and 'exit code -9' in line:
                    continue

                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)

                # Use insertHtml to properly render HTML entities
                self._log_append(self.full_control_status_text, html_line)

    def handle_joint_output(self, process):
        """Handle output for joint control processes (outputs to joint_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages
                if 'process has died' in line and 'exit code -9' in line:
                    continue

                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)

                # Use insertHtml to properly render HTML entities
                self._append_to_text_widget(self.joint_status_text, html_line)
 
    def _on_process_finished(self, process_key, button, name):
        """Handle when a process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.status_text, f"⚠ {name} exited")


    def _on_base_process_finished(self, process_key, button, name):
        """Handle when a base process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(self._get_base_process_start_text(process_key, name))
            button.setStyleSheet("")
            
            # Determine which status text to use
            if process_key.startswith('full'):
                status_text = self.full_control_status_text
            else:
                status_text = self.base_status_text
            
            self._log_append(status_text, f"{name} exited")

            if self._is_robot_bringup_process(process_key) and bool(button.property('uses_gazebo')):
                self._kill_gazebo_processes()
                button.setProperty('uses_gazebo', False)
            
            # Re-enable buttons disabled while localization was running
            if 'localization' in process_key:
                mode = 'full' if process_key.startswith('full') else 'base'
                for btn in self._get_buttons_to_disable_for_localization(mode):
                    btn.setEnabled(True)

            # Update tab states when base processes finish
            if not process_key.startswith('full') and self._is_base_tab_state_process(process_key):
                self._update_tab_states_for_base()

            elif process_key.startswith('full') and any(
                p in process_key
                for p in ['mobile_manipulator', 'localization', 'nav2']
            ):
                self._update_tab_states_for_full_control()
 
    def handle_output(self, process):
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Don't strip output to preserve formatting (especially for YAML)
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages (exit code -9 from our kill signals)
                if 'process has died' in line and 'exit code -9' in line:
                    continue
 
                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)
 
                # Use insertHtml to properly render HTML entities
                self._log_append(self.status_text, html_line)

    def handle_base_output(self, process):
        """Handle output for base control processes (outputs to base_status_text)"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Don't strip output to preserve formatting (especially for YAML)
            lines = output.split('\n')
            for line in lines:
                # Skip expected shutdown messages (exit code -9 from our kill signals)
                if 'process has died' in line and 'exit code -9' in line:
                    continue
 
                # Convert ANSI color codes to HTML
                html_line = self._ansi_to_html(line)
 
                # Use insertHtml to properly render HTML entities
                self._log_append(self.base_status_text, html_line)

    def _connect_robot_socket(self, status_text=None):
        """Connect to robot dashboard if not already connected"""
        if status_text is None:
            status_text = self.status_text

        context = 'full' if status_text is self.full_control_status_text else 'arm'
        desired_host = self._get_robot_ip_for_launch(context=context)
        if self.robot_host != desired_host:
            if self.robot_socket:
                try:
                    self.robot_socket.close()
                except Exception:
                    pass
                self.robot_socket = None
            self.robot_host = desired_host

        if self.robot_socket is None:
            try:
                self.robot_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.robot_socket.settimeout(5)
                self.robot_socket.connect((self.robot_host, self.robot_port))
                # Read initial connection message
                data = self.robot_socket.recv(1024)
                response = data.decode('utf-8').strip()
                self._log_append(status_text, f"✓ Connected to robot: {response}")
                return True
            except Exception as e:
                self._log_append(status_text, f"<span style='color: #f47067;'>✗ Failed to connect to robot: {e}</span>")
                self.robot_socket = None
                return False
        return True
 
    def _build_status_indicators(self, parent_layout):
        """Add Robot Mode / Safety Status / Program State indicator rows to parent_layout.

        Returns a dict keyed by command name ('robotmode', 'safetystatus',
        'programState') with (icon_label, value_label) tuples for later updates.
        """
        indicators = {}
        rows = [
            ('robotmode',    'Robot Mode'),
            ('safetystatus', 'Safety Status'),
            ('programState', 'Program State'),
        ]
        for key, label_text in rows:
            row = QHBoxLayout()
            icon = QLabel("○")
            icon.setStyleSheet("color: #8b949e; font-size: 16pt; font-weight: bold;")
            icon.setFixedWidth(24)
            icon.setAlignment(Qt.AlignCenter)
            name = QLabel(label_text + ":")
            name.setStyleSheet("font-weight: bold;")
            name.setFixedWidth(110)
            value = QLabel("—")
            value.setStyleSheet("color: #8b949e;")
            value.setWordWrap(True)
            row.addWidget(icon)
            row.addWidget(name)
            row.addWidget(value, 1)
            parent_layout.addLayout(row)
            indicators[key] = (icon, value)
        return indicators

    def _update_status_indicator(self, command, response, status_text):
        """Update icon/value labels based on the dashboard response."""
        if command not in ('robotmode', 'safetystatus', 'programState'):
            return
        if status_text is self.status_text:
            indicators = getattr(self, 'arm_status_indicators', None)
        elif status_text is getattr(self, 'full_control_status_text', None):
            indicators = getattr(self, 'full_status_indicators', None)
        else:
            indicators = None
        if not indicators or command not in indicators:
            return

        icon_label, value_label = indicators[command]
        text = (response or "").strip()
        GREEN = "#2ea043"
        RED = "#f47067"
        GREY = "#8b949e"

        if command == 'robotmode':
            val = text.split(':', 1)[-1].strip() if ':' in text else text
            ok = val.upper() == 'RUNNING'
            icon_label.setText("●")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText(val or "—")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")
        elif command == 'safetystatus':
            val = text.split(':', 1)[-1].strip() if ':' in text else text
            ok = val.upper() == 'NORMAL'
            icon_label.setText("☑" if ok else "⛔")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText(val or "—")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")
        elif command == 'programState':
            first = text.split()[0].upper() if text else ""
            ok = first == 'PLAYING'
            icon_label.setText("●")
            icon_label.setStyleSheet(
                f"color: {GREEN if ok else RED}; font-size: 16pt; font-weight: bold;"
            )
            value_label.setText("PLAYING" if ok else "STOPPED")
            value_label.setStyleSheet(f"color: {GREEN if ok else RED};")

    # Map of control commands to the status query(ies) that should be re-issued
    # afterwards so the indicators reflect the actual post-command robot state.
    _CONTROL_STATUS_REFRESH = {
        'power on':                 ['robotmode'],
        'power off':                ['robotmode'],
        'brake release':            ['robotmode'],
        'shutdown':                 ['robotmode'],
        'play':                     ['programState', 'robotmode'],
        'pause':                    ['programState'],
        'stop':                     ['programState'],
        'load Test_external_control.urp': ['programState'],
        'restart safety':           ['safetystatus', 'robotmode'],
        'close safety popup':       ['safetystatus'],
        'unlock protective stop':   ['safetystatus', 'robotmode'],
    }

    def _schedule_status_refresh(self, command, status_text):
        """If `command` is a control command we know about, query the related
        status(es) shortly afterwards so the indicators reflect the actual
        robot state (e.g. 'play' that fails should not leave PLAYING showing)."""
        refreshes = self._CONTROL_STATUS_REFRESH.get(command)
        if not refreshes:
            return

        def _do_refresh():
            for s in refreshes:
                self._send_robot_command(s, status_text=status_text)

        # Small delay so the robot has time to transition before we query.
        QTimer.singleShot(500, _do_refresh)

    def _send_robot_command(self, command, status_text=None):
        """Send command to robot dashboard and return response"""
        if status_text is None:
            status_text = self.status_text
        if not self._connect_robot_socket(status_text=status_text):
            return None

        try:
            self.robot_socket.send(str.encode(command + '\n'))
            self._log_append(status_text, f"<b style='color: #57ab5a;'>→ SENT: {command}</b>")
            self._log_append(status_text, "")  # Add newline after command

            data = self.robot_socket.recv(1024)
            response = data.decode('utf-8').strip()
            self._log_append(status_text, f"← RECV: {response}")
            self._update_status_indicator(command, response, status_text)
            self._schedule_status_refresh(command, status_text)
            return response
        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>✗ Command failed: {e}</span>")
            # Close socket on error so it reconnects next time
            if self.robot_socket:
                self.robot_socket.close()
                self.robot_socket = None
            return None
 
    def send_control_command(self, status_text=None):
        """Send selected control command to robot"""
        command = self.control_cmd_combo.currentText()
        self._send_robot_command(command, status_text=status_text)

    def send_all_full_control_status_commands(self):
        """Send all status commands sequentially (Full Control tab)"""
        status_commands = [
            'robotmode',
            'safetystatus',
            'programState',
            'running',
            'get loaded program',
            'is in remote control'
        ]
        
        self._log_append(self.full_control_status_text, "=" * 50)
        self._log_append(self.full_control_status_text, "📋 Sending all status commands...")
        self._log_append(self.full_control_status_text, "=" * 50)

        success_count = 0
        for command in status_commands:
            response = self._send_robot_command(command, status_text=self.full_control_status_text)
            if response is not None:
                success_count += 1
            # Add a small visual separator between commands
            self._log_append(self.full_control_status_text, "-" * 50)

        if success_count == len(status_commands):
            self._log_append(self.full_control_status_text, "✓ All status commands sent")
        elif success_count == 0:
            self._log_append(self.full_control_status_text, "✗ All status commands failed")
        else:
            self._log_append(self.full_control_status_text,
                f"⚠ {success_count}/{len(status_commands)} status commands succeeded"
            )
        self._log_append(self.full_control_status_text, "")
    
    def send_full_control_control_command(self):
        """Send selected control command to robot (Full Control tab)"""
        command = self.full_control_control_cmd_combo.currentText()
        self._send_robot_command(command, status_text=self.full_control_status_text)
    
    def run_full_control_ps_ros(self):
        """Run ps aux | grep ros2 command (Full Control tab)"""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        
        # Clear accumulator for new ps command
        self.ps_output_accumulator = ""
        
        # Connect to special handler that accumulates output
        process.readyReadStandardOutput.connect(lambda: self._handle_ps_output(process))
 
        # Display command in bold green
        cmd_str = 'ps aux | grep -E \'ros2|robot\' | grep -v grep'
        self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command using shell to support pipe
        process_key = 'full_ps_ros2'
        process.finished.connect(lambda: self._cleanup_full_ps_ros(process_key, process))
        process.start('bash', ['-c', 'ps aux | grep -E \'ros2|robot\' | grep -v grep'])
        self.process_map[process_key] = process
 
    def _handle_ps_output(self, process):
        """Handle output from ps command, accumulating it and displaying it"""
        output = process.readAllStandardOutput().data().decode()
        if output:
            # Accumulate for later parsing
            self.ps_output_accumulator += output
            
            # Also display it
            lines = output.split('\n')
            for line in lines:
                if line.strip():
                    html_line = self._ansi_to_html(line)
                    self._log_append(self.full_control_status_text, html_line)

    def _cleanup_full_ps_ros(self, process_key, process):
        """Clean up finished ps aux process and populate process combobox (Full Control tab)"""
        # Read any remaining output
        remaining_output = process.readAllStandardOutput().data().decode()
        if remaining_output:
            self.ps_output_accumulator += remaining_output
        
        # Parse the accumulated process list and populate combobox
        self.populate_process_combo(self.ps_output_accumulator)
        
        if process_key in self.process_map:
            del self.process_map[process_key]
    
    def _toggle_full_control_process(self, process_key, button, name, program, args):
        """Toggle a process on/off for Full Control tab"""
        if process_key in self.process_map:
            # Stop the process
            process = self.process_map[process_key]
            try:
                process.finished.disconnect()
            except:
                pass
 
            process.terminate()
            process.waitForFinished(3000)
            if process.state() == QProcess.Running:
                process.kill()
 
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.full_control_status_text, f"⏹ Stopped {name}")
        else:
            # Start the process
            process = QProcess(self)
            process.setProcessChannelMode(QProcess.MergedChannels)
            process.readyReadStandardOutput.connect(lambda: self.handle_full_control_output(process))
            process.finished.connect(lambda: self._on_full_control_process_finished(process_key, button, name))

            # Display command in bold green
            cmd_str = program + ' ' + ' '.join(args)
            self._log_append(self.full_control_status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
 
            process.start(program, args)
            self.process_map[process_key] = process
            button.setText(f"Stop {name}")
            button.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
    
    def _on_full_control_process_finished(self, process_key, button, name):
        """Handle when a Full Control process finishes unexpectedly"""
        if process_key in self.process_map:
            del self.process_map[process_key]
            button.setText(f"Start {name}")
            button.setStyleSheet("")
            self._log_append(self.full_control_status_text, f"⚠ {name} exited")

    def list_controllers(self):
        """List controllers for all detected controller_manager nodes (Arm tab)."""
        process = QProcess(self)
        process.setProcessChannelMode(QProcess.MergedChannels)
        process.readyReadStandardOutput.connect(lambda: self.handle_output(process))
 
        # Display command in bold green
        cmd_str = "for cm in $(ros2 node list | grep -E '(^|/)controller_manager$'); do ros2 control list_controllers -c $cm; done"
        self._log_append(self.status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Run the command
        process_key = 'list_controllers'
        process.finished.connect(lambda: self._cleanup_list_controllers(process_key))
        process.start('bash', ['-c', self._build_list_all_controllers_script()])
        self.process_map[process_key] = process
 
    def _cleanup_list_controllers(self, process_key):
        """Clean up finished list controllers process"""
        if process_key in self.process_map:
            exit_code = self.process_map[process_key].exitCode()
            del self.process_map[process_key]
            if exit_code != 0:
                self._log_append(self.status_text, f"<span style='color: #c69026;'>⚠ List controllers command finished with exit code {exit_code}</span>")
            else:
                self._log_append(self.status_text, "✓ List controllers command completed")
 
    def clear_status(self):
        widget = self.status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_status.setVisible(True)

    def restore_status(self):
        widget = self.status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_status.setVisible(False)

    def clear_base_status(self):
        widget = self.base_status_text
        self.tab_log_backups[id(widget)] = list(self.tab_log_entries.get(id(widget), []))
        self.tab_log_entries[id(widget)] = []
        widget.clear()
        self.btn_restore_base_status.setVisible(True)

    def restore_base_status(self):
        widget = self.base_status_text
        backup = self.tab_log_backups.get(id(widget))
        if backup:
            current = self.tab_log_entries.get(id(widget), [])
            self.tab_log_entries[id(widget)] = backup + current
            self.tab_log_backups[id(widget)] = []
            self._log_apply_filter(widget)
            self.btn_restore_base_status.setVisible(False)

    def clear_joint_status(self):
        """Clear joint control status text"""
        self.joint_status_text.clear()
    def send_gantry_command(self):
        """Publish the current gantry slider targets to the gantry position controller."""
        data = [float(self.gantry_inputs[j].value()) for j in self.gantry_joint_names]
        msg = Float64MultiArray()
        msg.data = data
        self.gantry_cmd_publisher.publish(msg)
        self._log_append(
            self.joint_status_text,
            "<b style='color: #539bf5;'>&#9654; publish /gantry_position_controller/commands</b> "
            f"[Z={data[0]:.3f} m, Y={data[1]:.3f} m, X={data[2]:.3f} m, ROT={data[3]:.3f} rad]"
        )

    def home_gantry(self):
        """Reset all gantry sliders to zero and send the home command."""
        for j in self.gantry_joint_names:
            self.gantry_inputs[j].setValue(0.0)
        self.send_gantry_command()

    def reset_planner(self):
        """Reset the planner from Full Control tab"""
        self._reset_planner_generic('reset_planner_full', self.full_control_status_text, self.handle_full_control_output)

    def reset_octomap(self):
        """Reset the octomap from Full Control tab"""
        self._reset_octomap_generic('reset_octomap_full', self.full_control_status_text, self.handle_full_control_output)

    def reset_planner_joint(self):
        """Reset the planner from Joint Control tab"""
        self._reset_planner_generic('reset_planner_joint', self.joint_status_text, self.handle_joint_output)

    def reset_octomap_joint(self):
        """Reset the octomap from Joint Control tab"""
        self._reset_octomap_generic('reset_octomap_joint', self.joint_status_text, self.handle_joint_output)
    
    def _reset_planner_generic(self, process_key, status_text_widget, output_handler):
        """Generic method to reset the planner by publishing to /planner/reset topic"""
        # Clean up existing process if any
        if process_key in self.process_map:
            existing_process = self.process_map[process_key]
            if existing_process.state() == QProcess.Running:
                existing_process.kill()
                existing_process.waitForFinished(1000)
            del self.process_map[process_key]
        
        # Create and start the process
        process = QProcess()
        command = 'ros2'
        args = ['topic', 'pub', '--once', '/planner/reset', 'std_msgs/msg/Bool', '{data: true}']
        
        # Display command in bold green
        cmd_str = command + ' ' + ' '.join(args)
        self._log_append(status_text_widget, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")
        
        # Connect output handler
        process.readyReadStandardOutput.connect(lambda: output_handler(process))
        process.readyReadStandardError.connect(lambda: output_handler(process))
        process.finished.connect(lambda: self._cleanup_reset_planner(process_key, status_text_widget))
        
        # Start the process
        process.start(command, args)
        self.process_map[process_key] = process
    
    def _cleanup_reset_planner(self, process_key, status_text_widget):
        """Clean up reset planner process"""
        if process_key in self.process_map:
            process = self.process_map[process_key]
            exit_code = process.exitCode()
            if exit_code == 0:
                self._append_to_text_widget(
                    status_text_widget,
                    f"<span style='color: #3fb950;'>[Reset Planner]</span> Successfully reset planner."
                )
            else:
                self._append_to_text_widget(
                    status_text_widget,
                    f"<span style='color: #f85149;'>[Reset Planner]</span> Command failed with exit code {exit_code}."
                )
            del self.process_map[process_key]

    def _reset_octomap_generic(self, process_key, status_text_widget, output_handler):
        """Generic method to reset octomap_server by calling its reset service"""
        if process_key in self.process_map:
            existing_process = self.process_map[process_key]
            if existing_process.state() == QProcess.Running:
                existing_process.kill()
                existing_process.waitForFinished(1000)
            del self.process_map[process_key]

        process = QProcess()
        command = 'ros2'
        args = ['service', 'call', '/octomap_server/reset', 'std_srvs/srv/Empty', '{}']

        cmd_str = command + ' ' + ' '.join(args)
        self._log_append(status_text_widget, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        process.readyReadStandardOutput.connect(lambda: output_handler(process))
        process.readyReadStandardError.connect(lambda: output_handler(process))
        process.finished.connect(lambda: self._cleanup_reset_octomap(process_key, status_text_widget))

        process.start(command, args)
        self.process_map[process_key] = process

    def _cleanup_reset_octomap(self, process_key, status_text_widget):
        """Clean up reset octomap process"""
        if process_key in self.process_map:
            process = self.process_map[process_key]
            exit_code = process.exitCode()
            if exit_code == 0:
                self._append_to_text_widget(
                    status_text_widget,
                    f"<span style='color: #3fb950;'>[Reset Octomap]</span> Successfully reset octomap."
                )
            else:
                self._append_to_text_widget(
                    status_text_widget,
                    (
                        f"<span style='color: #d29922;'>[Reset Octomap]</span> "
                        f"Reset command exited with code {exit_code}. "
                        "If octomap_server restarted while handling the reset, wait a few seconds for the respawned node to come back up and then retry if needed."
                    )
                )
                self._append_to_text_widget(
                    status_text_widget,
                    "<span style='color: #8b949e;'>[Reset Octomap]</span> Automatic respawn is enabled for octomap_server in the launch configuration."
                )
            del self.process_map[process_key]

    def _get_joint_states_topic_for_ui(self, context='arm'):
        """Pick the joint_states topic for slider updates/readback based on context and planner backend."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            if planner_backend == 'moveit':
                return '/arm/joint_states'
        return '/joint_states'

    def _get_planned_trajectory_topic_for_ui(self):
        """Pick the planned_trajectory topic for Joint Control publishing."""
        return '/planned_trajectory'

    def _get_robot_ip_for_launch(self, context='arm'):
        """IP address for the UR robot."""
        return '192.168.1.102'

    def _get_planner_backend(self, context='full'):
        """Pick planner_backend from the Full Control tab (defaults to legacy)."""
        if hasattr(self, 'full_control_planner_backend_combo'):
            return self.full_control_planner_backend_combo.currentText()
        return 'legacy'

    def _get_moveit_planning_pipeline(self, context='full'):
        """Pick the MoveIt planning pipeline from the Full Control tab."""
        if hasattr(self, 'full_control_moveit_pipeline_combo'):
            return self.full_control_moveit_pipeline_combo.currentText()
        return 'pilz_industrial_motion_planner'

    def _get_moveit_planner_id(self, context='full'):
        """Pick the MoveIt planner id from the Full Control tab."""
        if hasattr(self, 'full_control_moveit_planner_id_combo'):
            return self.full_control_moveit_planner_id_combo.currentText()
        return 'PTP'

    def _get_moveit_launch_args(self, context='arm'):
        """Build launch arguments for the selected MoveIt pipeline and planner id."""
        if self._get_planner_backend(context=context) != 'moveit':
            return []

        planning_pipeline = self._get_moveit_planning_pipeline(context=context)
        moveit_args = [f'moveit_planning_pipeline:={planning_pipeline}']

        if planning_pipeline == 'pilz_industrial_motion_planner':
            pose_planner_id = self._get_moveit_planner_id(context=context)
            moveit_args.append(f'moveit_pose_planner_id:={pose_planner_id}')
            moveit_args.append('moveit_joint_planner_id:=PTP')
        else:
            moveit_args.append('moveit_pose_planner_id:=RRTConnectkConfigDefault')
            moveit_args.append('moveit_joint_planner_id:=RRTConnectkConfigDefault')

        return moveit_args

    def _get_namespace_for_arm_launch(self, planner_backend):
        """Arm launches now stay in the root namespace for both backends."""
        return ''

    def _get_robot_ip_for_arm_launch(self):
        """Backward-compatible arm launch helper."""
        return self._get_robot_ip_for_launch(context='arm')

    def _get_emergency_stop_topic_for_ui(self, context='arm'):
        """Pick the emergency_stop topic based on context and planner backend."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            if planner_backend == 'moveit':
                return '/arm/emergency_stop'
        return '/emergency_stop'

    def _get_status_text_for_context(self, context='arm'):
        """Get the appropriate status text widget based on context."""
        if context == 'full':
            return self.full_control_status_text
        return self.status_text

    def _get_emergency_stop_button_for_context(self, context='arm'):
        """Get the appropriate emergency stop button based on context."""
        if context == 'full' and hasattr(self, 'btn_full_control_emergency_stop'):
            return self.btn_full_control_emergency_stop
        return self.btn_emergency_stop

    def _get_send_position_service_name(self, context='arm'):
        """Pick send_position service name based on the active tab and planner backend."""
        if context == 'arm':
            planner_backend = self._get_planner_backend(context='arm')
            namespace = self._get_namespace_for_arm_launch(planner_backend)
            if namespace:
                return f'/{namespace}/send_position'
        return '/send_position'

    def _cleanup_send_position_service_call(self, process_key, position_name, status_text):
        """Clean up one-shot send position service call process."""
        if process_key not in self.process_map:
            return

        exit_code = self.process_map[process_key].exitCode()
        del self.process_map[process_key]

        if exit_code == 0:
            self._log_append(status_text, f"✓ Position '{position_name}' request completed")
        else:
            self._log_append(status_text,
                f"<span style='color: #c69026;'>⚠ Position '{position_name}' request finished with exit code {exit_code}</span>"
            )
 
    def emergency_stop(self, context='arm'):
        """Toggle emergency stop state (latched)."""
        self.set_emergency_stop_state(not self.emergency_stop_active, source="ui", context=context)
            
    def _on_emergency_stop_state(self, msg: Bool):
        """Sync UI with the latched /arm/emergency_stop state (even if published externally)."""
        self.set_emergency_stop_state(bool(msg.data), source="topic", context='arm')

    def set_emergency_stop_state(self, active: bool, source: str = "ui", context: str = "arm"):
        """
        Centralized state setter.

        Behavior (matches old behavior):
        - Only publishes to emergency_stop topic when source == "ui"
        - Only publishes when the state actually changes
        - Topic callbacks only update UI (no re-publish), preventing feedback loops
        - Topic namespace depends on context and simulation settings
        """
        # If no change, do nothing (prevents repeated publishes / UI churn)
        if active == getattr(self, "emergency_stop_active", False):
            return

        # Update internal state + UI
        self.emergency_stop_active = active
        self._update_emergency_stop_button_ui(context=context)
        # QApplication.processEvents()

        # If this came from the topic, do not publish or trigger side effects
        if source != "ui":
            return

        # Get the appropriate status text widget for this context
        status_text = self._get_status_text_for_context(context)

        if not rclpy.ok():
            self._log_append(status_text, "<span style='color: #c69026;'>⚠ ROS context invalid - cannot set emergency stop</span>")
            return

        # Determine the correct topic based on context and simulation settings
        emergency_stop_topic = self._get_emergency_stop_topic_for_ui(context)

        # Display the command being executed in green color
        action_str = "true" if active else "false"
        cmd_str = f'ros2 topic pub --once {emergency_stop_topic} std_msgs/msg/Bool "{{data: {action_str}}}"'
        self._log_append(status_text, f"<b style='color: #57ab5a;'>▶ {cmd_str}</b>")

        # Publish only on user-triggered change
        try:
            stop_msg = Bool()
            stop_msg.data = active

            # Get or create a persistent publisher for the determined topic
            publisher = self._emergency_stop_publishers.get(emergency_stop_topic)
            if publisher is None:
                qos_profile = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.TRANSIENT_LOCAL,
                    history=HistoryPolicy.KEEP_LAST,
                    depth=10,
                )
                publisher = self.node.create_publisher(Bool, emergency_stop_topic, qos_profile)
                self._emergency_stop_publishers[emergency_stop_topic] = publisher

            publisher.publish(stop_msg)

        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>❌ Failed to publish {emergency_stop_topic}: {e}</span>")
            return

        # User-triggered side-effects only
        try:
            if active:
                self._log_append(status_text, "<span style='color: #c69026; font-weight: bold;'>⚠ EMERGENCY STOP - Published stop signal</span>")

                if self.current_goal_handle is not None:
                    self.current_goal_handle.cancel_goal_async()
                    self._log_append(status_text, "<span style='color: #c69026;'>⚠ Canceling current trajectory goal...</span>")
                    self.current_goal_handle = None

                response = self._send_robot_command('stop', status_text=status_text)
                if response:
                    self._log_append(status_text, "<span style='color: #c69026; font-weight: bold;'>⚠ Robot protective stop triggered</span>")

            else:
                self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ EMERGENCY STOP RELEASED - Published release signal</span>")

                self._send_robot_command('close safety popup', status_text=status_text)
                response = self._send_robot_command('unlock protective stop', status_text=status_text)
                if response:
                    self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ Robot protective stop released (requested)</span>")

                    play_resp = self._send_robot_command('play', status_text=status_text)
                    if play_resp:
                        self._log_append(status_text, "<span style='color: #57ab5a; font-weight: bold;'>✓ Robot program started (play)</span>")

        except Exception as e:
            self._log_append(status_text, f"<span style='color: #f47067;'>❌ Error while applying emergency stop state: {e}</span>")


    def _update_emergency_stop_button_ui(self, context='arm'):
        """Update button label + color according to emergency stop state.
        Updates both buttons to keep them in sync."""
        if self.emergency_stop_active:
            text = "EMERGENCY STOP ACTIVE (Click to Release)"
            style = "background-color: #8b0000; color: white; font-weight: bold;"
            checked = True
        else:
            text = "EMERGENCY STOP (Click to Activate)"
            style = "background-color: red; color: white; font-weight: bold;"
            checked = False

        # Update Arm Control button
        if hasattr(self, "btn_emergency_stop"):
            self.btn_emergency_stop.setText(text)
            self.btn_emergency_stop.setStyleSheet(style)
            self.btn_emergency_stop.setChecked(checked)

        # Update Full Control button
        if hasattr(self, "btn_full_control_emergency_stop"):
            self.btn_full_control_emergency_stop.setText(text)
            self.btn_full_control_emergency_stop.setStyleSheet(style)
            self.btn_full_control_emergency_stop.setChecked(checked)
 
    def closeEvent(self, event):
        self.timer.stop()

        # Detach every QProcess from this window so Qt does NOT send SIGTERM
        # to the underlying OS processes when the window is destroyed.
        for process in list(self.process_map.values()):
            try:
                process.setParent(None)
            except Exception:
                pass
        self.process_map.clear()

        for proc in (self.fsm_launch_process, self.fsm_node_process):
            if proc is not None:
                try:
                    proc.setParent(None)
                except Exception:
                    pass
        self.fsm_launch_process = None
        self.fsm_node_process = None

        # Close robot socket if connected
        if self.robot_socket:
            try:
                self.robot_socket.close()
            except Exception:
                pass

        self.node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        event.accept()

    def _cleanup_ros_children(self):
        """Best-effort cleanup of ROS child processes launched by ros2 launch/run. Only called on window close."""
        patterns = [
            'rviz2',
            'ros2_control_node',
            'publisher_joint_trajectory_planned',
            'planner_node',
            'end_effector_pose_node',
            'position_sender_node',
            'align_ee_to_wall',
            'robot_state_publisher',
        ]
        for pat in patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pat], timeout=1, stderr=subprocess.DEVNULL)
            except:
                pass
 
    def _cleanup_ros_children_of_pid(self, parent_pid):
        """Kill specific ROS child processes that are descendants of the given parent PID."""
        if not parent_pid:
            return
 
        # Get all child PIDs recursively
        try:
            result = subprocess.run(['pgrep', '-P', str(parent_pid)], 
                                  capture_output=True, text=True, timeout=1)
            child_pids = result.stdout.strip().split('\n')
 
            for child_pid in child_pids:
                if child_pid:
                    # Recursively kill children's children
                    self._cleanup_ros_children_of_pid(int(child_pid))
                    # Kill this child
                    try:
                        subprocess.run(['kill', '-9', child_pid], timeout=1, stderr=subprocess.DEVNULL)
                    except:
                        pass
        except:
            pass

    def _kill_gazebo_processes(self):
        """Kill newer Gazebo (Ignition/gz) processes."""
        # List of Gazebo/Ignition process patterns to kill
        gz_patterns = [
            'gz sim',
            'ign gazebo',
            'ruby.*gz',
            'gzserver',
            'gz-sim',
        ]
        
        for pattern in gz_patterns:
            try:
                subprocess.run(['pkill', '-9', '-f', pattern], timeout=2, stderr=subprocess.DEVNULL)
            except:
                pass
    
    def populate_process_combo(self, ps_output):
        """Parse ps output and populate the process combobox"""
        self.current_process_list = []
        self.process_combo.clear()
        
        if not ps_output:
            self.process_combo.addItem("No processes found")
            return
        
        lines = ps_output.strip().split('\n')
        item_index = 0
        for line in lines:
            if not line.strip():
                continue
            
            # Parse ps aux output: USER PID %CPU %MEM VSZ RSS TTY STAT START TIME COMMAND
            parts = line.split(None, 10)  # Split into max 11 parts
            if len(parts) >= 11:
                pid = parts[1]
                command = parts[10]
                
                # Store process info and add to combobox
                process_info = {'pid': pid, 'command': command, 'full_line': line}
                self.current_process_list.append(process_info)
                
                # Display only PID in the combobox
                display_text = f"PID: {pid}"
                self.process_combo.addItem(display_text)
                
                # Set tooltip to show full command
                tooltip = f"PID: {pid}\nCommand: {command}"
                self.process_combo.setItemData(item_index, tooltip, Qt.ToolTipRole)
                item_index += 1
        
        if not self.current_process_list:
            self.process_combo.addItem("No processes found")
        else:
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #57ab5a;'>Found {len(self.current_process_list)} ROS2 processes</span>")

    def kill_selected_process(self):
        """Kill the process selected in the combobox"""
        current_index = self.process_combo.currentIndex()

        if current_index < 0 or current_index >= len(self.current_process_list):
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No process selected or invalid selection</span>")
            return

        process_info = self.current_process_list[current_index]
        pid = process_info['pid']
        command = process_info['command']

        # Confirm and kill
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            'Confirm Kill Process',
            f"Are you sure you want to kill process {pid}?\n\nCommand: {command[:100]}",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            # Execute kill command
            kill_process = QProcess(self)
            kill_process.setProcessChannelMode(QProcess.MergedChannels)

            cmd_str = f'kill -9 {pid}'
            self._log_append(self.full_control_status_text,
                             f"<b style='color: #d73a49;'>▶ {cmd_str}</b>")

            kill_process.finished.connect(lambda: self._on_kill_process_finished(pid, kill_process))
            kill_process.start('kill', ['-9', pid])

    def _on_kill_process_finished(self, pid, process):
        """Handle kill process completion"""
        exit_code = process.exitCode()

        if exit_code == 0:
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #57ab5a;'>✓ Successfully killed process {pid}</span>")
            # Refresh the process list after killing
            QTimer.singleShot(500, self.run_full_control_ps_ros)
        else:
            error_output = process.readAllStandardOutput().data().decode()
            self._log_append(self.full_control_status_text,
                             f"<span style='color: #d73a49;'>✗ Failed to kill process {pid}: {error_output}</span>")

    def kill_all_processes(self):
        """Kill all detected processes except UI itself and ros2 daemon"""
        if not self.current_process_list:
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No processes to kill</span>")
            return

        # Get our own PID to exclude
        ui_pid = os.getpid()

        # Filter processes to kill (exclude UI and ros2 daemon)
        processes_to_kill = []
        for process_info in self.current_process_list:
            pid = process_info['pid']
            command = process_info['command'].lower()

            # Skip our own process
            if int(pid) == ui_pid:
                continue

            # Skip ros2 daemon
            if 'ros2' in command and 'daemon' in command:
                continue

            # Skip if it's the UI.py script
            if 'ui.py' in command:
                continue

            processes_to_kill.append(process_info)

        if not processes_to_kill:
            self._log_append(self.full_control_status_text,
                             "<span style='color: #d73a49;'>No processes to kill (all are protected)</span>")
            return

        # Confirm kill all
        from PyQt5.QtWidgets import QMessageBox
        reply = QMessageBox.question(
            self,
            'Confirm Kill All Processes',
            f"Are you sure you want to kill {len(processes_to_kill)} process(es)?\n\n"
            f"This will kill all detected processes except the UI and ros2 daemon.",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No
        )

        if reply == QMessageBox.Yes:
            killed_count = 0
            failed_count = 0

            self._log_append(self.full_control_status_text,
                             f"<b style='color: #d73a49;'>▶ Killing {len(processes_to_kill)} process(es)...</b>")

            for process_info in processes_to_kill:
                pid = process_info['pid']
                command = process_info['command']

                try:
                    # Use os.kill for synchronous killing
                    os.kill(int(pid), signal.SIGKILL)
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #57ab5a;'>✓ Killed PID {pid}: {command[:60]}...</span>")
                    killed_count += 1
                except ProcessLookupError:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #e3b341;'>⚠ Process {pid} already terminated</span>")
                    failed_count += 1
                except PermissionError:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #d73a49;'>✗ Permission denied for PID {pid}</span>")
                    failed_count += 1
                except Exception as e:
                    self._log_append(self.full_control_status_text,
                                     f"<span style='color: #d73a49;'>✗ Failed to kill PID {pid}: {str(e)}</span>")
                    failed_count += 1

            self._log_append(self.full_control_status_text,
                             f"<b style='color: #57ab5a;'>Finished: {killed_count} killed, {failed_count} failed</b>")
            
            # Refresh the process list after killing
            QTimer.singleShot(500, self.run_full_control_ps_ros)
 
    # ───────────────────────── FSM TAB ─────────────────────────

    def _create_fsm_tab(self):
        """Create the FSM control tab."""
        fsm_tab = QWidget()
        fsm_tab_layout = QVBoxLayout(fsm_tab)

        # ── Controls row ──
        controls_row = QHBoxLayout()

        controls_row.addWidget(QLabel("Sim Mode:"))
        self.fsm_sim_combo = QComboBox()
        self.fsm_sim_combo.addItems(["true", "false"])
        controls_row.addWidget(self.fsm_sim_combo)

        controls_row.addSpacing(16)
        controls_row.addWidget(QLabel("Initial State:"))
        self.fsm_state_combo = QComboBox()
        self.fsm_state_combo.addItems([
            "Initialization", "ReceiveNav2Map", "GetSemanticMap", "WaitForData",
            "TargetSelection", "ManipulatorFolding", "BasePlacementComputation",
            "NavigateToTarget", "ManipulatorReachability", "NearbyPointSelection",
            "ManipulatorUnfolding", "DrillApproach", "SuctionDrillStart", "Drilling",
            "TakeOutDrill", "SuctionDrillStop", "DrillRetract", "SampleScanning",
            "StoringToDatabase", "HomePosition", "Finished", "Error",
        ])
        controls_row.addWidget(self.fsm_state_combo)

        controls_row.addSpacing(24)
        self.btn_fsm_start = QPushButton("Start FSM")
        self.btn_fsm_start.clicked.connect(self._toggle_fsm)
        controls_row.addWidget(self.btn_fsm_start)

        controls_row.addStretch()
        fsm_tab_layout.addLayout(controls_row)

        # ── Status header ──
        fsm_status_header = QHBoxLayout()
        fsm_status_header.addWidget(QLabel("FSM Output"))
        fsm_status_header.addStretch()

        fsm_status_header.addWidget(QLabel("Filter:"))
        self.fsm_filter_input = QLineEdit()
        self.fsm_filter_input.setPlaceholderText("Filter output lines...")
        self.fsm_filter_input.setMaximumWidth(200)
        self.fsm_filter_input.textChanged.connect(self._fsm_apply_filter)
        fsm_status_header.addWidget(self.fsm_filter_input)

        self.btn_restore_fsm_status = QPushButton("Restore")
        self.btn_restore_fsm_status.clicked.connect(self.restore_fsm_status)
        self.btn_restore_fsm_status.setMaximumWidth(80)
        self.btn_restore_fsm_status.setVisible(False)
        fsm_status_header.addWidget(self.btn_restore_fsm_status)

        btn_clear_fsm_status = QPushButton("Clear")
        btn_clear_fsm_status.clicked.connect(self.clear_fsm_status)
        btn_clear_fsm_status.setMaximumWidth(80)
        fsm_status_header.addWidget(btn_clear_fsm_status)

        fsm_tab_layout.addLayout(fsm_status_header)

        # ── Status output (full width, wrapped) ──
        self.fsm_status_text = QTextEdit()
        self.fsm_status_text.setReadOnly(True)
        self.fsm_status_text.setAcceptRichText(True)
        self.fsm_status_text.setLineWrapMode(QTextEdit.WidgetWidth)
        self.fsm_status_text.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        self.fsm_status_text.setStyleSheet(
            "background-color: #22272e; color: #adbac7; border: 1px solid #444c56; "
            "font-family: 'Courier New', monospace; white-space: pre-wrap;"
        )
        font_metrics = QFontMetrics(self.fsm_status_text.font())
        self.fsm_status_text.setTabStopDistance(font_metrics.horizontalAdvance(' ') * 8)
        fsm_tab_layout.addWidget(self.fsm_status_text, 1)

        # ── Stdin input row ──
        stdin_row = QHBoxLayout()
        stdin_row.addWidget(QLabel("Send input:"))
        self.fsm_stdin_input = QLineEdit()
        self.fsm_stdin_input.setPlaceholderText("Type input for the running FSM process and press Enter...")
        self.fsm_stdin_input.setEnabled(False)
        self.fsm_stdin_input.returnPressed.connect(self._send_fsm_input)
        stdin_row.addWidget(self.fsm_stdin_input)
        self.btn_fsm_send_input = QPushButton("Send")
        self.btn_fsm_send_input.setMaximumWidth(70)
        self.btn_fsm_send_input.setEnabled(False)
        self.btn_fsm_send_input.clicked.connect(self._send_fsm_input)
        stdin_row.addWidget(self.btn_fsm_send_input)
        fsm_tab_layout.addLayout(stdin_row)

        return fsm_tab

    def _toggle_fsm(self):
        """Start or stop the FSM launch + node pair."""
        if self.fsm_launch_process is not None or self.fsm_node_process is not None:
            self._stop_fsm()
        else:
            self._start_fsm()

    def _start_fsm(self):
        sim = self.fsm_sim_combo.currentText()
        state = self.fsm_state_combo.currentText()

        self.btn_fsm_start.setText("Stop FSM")
        self.btn_fsm_start.setStyleSheet("background-color: #4CAF50; color: white; font-weight: bold;")
        self._fsm_set_input_enabled(True)

        # ── Step 1: launch file ──
        launch_args = ['launch', 'task_planner_fsm', 'task_planner.launch.py']
        self._fsm_append_log(
            f"<b style='color: #57ab5a;'>▶ ros2 {' '.join(launch_args)}</b>",
            f"▶ ros2 {' '.join(launch_args)}",
        )
        proc_launch = QProcess(self)
        proc_launch.setProcessChannelMode(QProcess.MergedChannels)
        proc_launch.readyReadStandardOutput.connect(
            lambda p=proc_launch: self._handle_fsm_output(p)
        )
        proc_launch.finished.connect(
            lambda code, status, p=proc_launch: self._on_fsm_launch_finished(p, code)
        )
        proc_launch.start('ros2', launch_args)
        self.fsm_launch_process = proc_launch

        # ── Step 2: fsm_node (delayed so launch has time to spin up) ──
        node_args = [
            'run', 'task_planner_fsm', 'fsm_node',
            '--sim', sim,
            '--initial-state', state,
        ]
        QTimer.singleShot(3000, lambda: self._start_fsm_node(node_args))

    def _start_fsm_node(self, node_args):
        """Start the fsm_node process (called after launch delay)."""
        if self.fsm_launch_process is None:
            # Launch was already stopped before the timer fired
            return
        self._fsm_append_log(
            f"<b style='color: #57ab5a;'>▶ ros2 {' '.join(node_args)}</b>",
            f"▶ ros2 {' '.join(node_args)}",
        )
        proc_node = QProcess(self)
        proc_node.setProcessChannelMode(QProcess.MergedChannels)
        proc_node.readyReadStandardOutput.connect(
            lambda p=proc_node: self._handle_fsm_output(p)
        )
        proc_node.finished.connect(
            lambda code, status, p=proc_node: self._on_fsm_node_finished(p, code)
        )
        proc_node.start('ros2', node_args)
        self.fsm_node_process = proc_node

    def _stop_fsm(self):
        """Stop both FSM processes and their entire spawned process trees."""

        # ── Phase 1: collect all descendant PIDs BEFORE sending any signals.
        # Once a parent is killed its children are reparented to init (PID 1),
        # so pgrep -P <dead_parent> returns nothing. We must snapshot the tree now.
        def _collect_descendants(parent_pid):
            pids = []
            if not parent_pid:
                return pids
            try:
                r = subprocess.run(
                    ['pgrep', '-P', str(parent_pid)],
                    capture_output=True, text=True, timeout=1,
                )
                for s in r.stdout.strip().split('\n'):
                    if s.strip():
                        child = int(s)
                        pids.extend(_collect_descendants(child))
                        pids.append(child)
            except Exception:
                pass
            return pids

        procs = [p for p in (self.fsm_node_process, self.fsm_launch_process) if p is not None]
        proc_pids = []
        all_descendants = []
        for proc in procs:
            try:
                proc.finished.disconnect()
            except Exception:
                pass
            pid = proc.processId()
            proc_pids.append(pid)
            if pid:
                all_descendants.extend(_collect_descendants(pid))

        # ── Phase 2: graceful SIGINT to parent processes.
        for pid in proc_pids:
            if pid:
                try:
                    os.kill(pid, signal.SIGINT)
                except Exception:
                    pass
        for proc in procs:
            proc.waitForFinished(3000)

        # ── Phase 3: SIGKILL every collected descendant.
        for dpid in all_descendants:
            try:
                os.kill(dpid, signal.SIGKILL)
            except Exception:
                pass

        # ── Phase 4: force-kill the parent QProcesses if still alive.
        for proc in procs:
            if proc.state() == QProcess.Running:
                proc.terminate()
                proc.waitForFinished(1000)
            if proc.state() == QProcess.Running:
                proc.kill()
                proc.waitForFinished(1000)
            proc.deleteLater()

        # ── Phase 5: wipe out Gazebo and any remaining stragglers.
        self._kill_gazebo_processes()

        self.fsm_node_process = None
        self.fsm_launch_process = None
        self.btn_fsm_start.setText("Start FSM")
        self.btn_fsm_start.setStyleSheet("")
        self._fsm_set_input_enabled(False)
        self._fsm_append_log(
            "<span style='color: #e3b341;'>⏹ FSM stopped</span>",
            "⏹ FSM stopped",
        )

    def _handle_fsm_output(self, process):
        """Stream output from a FSM process into the status pane."""
        output = process.readAllStandardOutput().data().decode(errors='replace')
        if not output:
            return
        for line in output.split('\n'):
            if line.strip():
                self._fsm_append_log(self._ansi_to_html(line), line)

    def _on_fsm_launch_finished(self, process, exit_code):
        if process is not self.fsm_launch_process:
            return
        self.fsm_launch_process = None
        color = '#57ab5a' if exit_code == 0 else '#f47067'
        msg = f"Launch exited (code {exit_code})"
        self._fsm_append_log(f"<span style='color: {color};'>{msg}</span>", msg)
        process.deleteLater()
        if self.fsm_node_process is None:
            self.btn_fsm_start.setText("Start FSM")
            self.btn_fsm_start.setStyleSheet("")
            self._fsm_set_input_enabled(False)

    def _on_fsm_node_finished(self, process, exit_code):
        if process is not self.fsm_node_process:
            return
        self.fsm_node_process = None
        color = '#57ab5a' if exit_code == 0 else '#f47067'
        msg = f"fsm_node exited (code {exit_code})"
        self._fsm_append_log(f"<span style='color: {color};'>{msg}</span>", msg)
        process.deleteLater()
        if self.fsm_launch_process is None:
            self.btn_fsm_start.setText("Start FSM")
            self.btn_fsm_start.setStyleSheet("")
            self._fsm_set_input_enabled(False)

    def _fsm_set_input_enabled(self, enabled: bool):
        self.fsm_stdin_input.setEnabled(enabled)
        self.btn_fsm_send_input.setEnabled(enabled)
        if not enabled:
            self.fsm_stdin_input.clear()

    def _send_fsm_input(self):
        text = self.fsm_stdin_input.text()
        if not text:
            return
        # Prefer the node process; fall back to the launch process
        target = self.fsm_node_process or self.fsm_launch_process
        if target is None or target.state() != QProcess.Running:
            self._fsm_append_log(
                "<span style='color: #f47067;'>⚠ No running FSM process to send input to</span>",
                "⚠ No running FSM process to send input to",
            )
            return
        target.write((text + '\n').encode())
        self._fsm_append_log(
            f"<span style='color: #76e3ea;'>▷ {html.escape(text)}</span>",
            f"▷ {text}",
        )
        self.fsm_stdin_input.clear()

    def _fsm_append_log(self, html_content, plain_text, add_newline=True):
        """Store entry in the log buffer and display it if it passes the current filter."""
        self.fsm_log_entries.append((html_content, plain_text, add_newline))
        term = self.fsm_filter_input.text().strip().lower()
        if not term or term in plain_text.lower():
            self._append_to_text_widget(self.fsm_status_text, html_content, add_newline)

    def _fsm_apply_filter(self):
        """Rebuild the status pane showing only lines that match the filter text."""
        term = self.fsm_filter_input.text().strip().lower()
        scrollbar = self.fsm_status_text.verticalScrollBar()
        was_at_bottom = scrollbar.value() >= scrollbar.maximum() - 10
        self.fsm_status_text.clear()
        for h, plain, nl in self.fsm_log_entries:
            if not term or term in plain.lower():
                self._append_to_text_widget(self.fsm_status_text, h, nl)
        if was_at_bottom:
            scrollbar.setValue(scrollbar.maximum())

    def clear_fsm_status(self):
        self.fsm_log_backup = list(self.fsm_log_entries)
        self.fsm_log_entries.clear()
        self.fsm_status_text.clear()
        self.btn_restore_fsm_status.setVisible(True)

    def restore_fsm_status(self):
        if self.fsm_log_backup:
            self.fsm_log_entries = self.fsm_log_backup + self.fsm_log_entries
            self.fsm_log_backup = []
            self._fsm_apply_filter()
            self.btn_restore_fsm_status.setVisible(False)


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    window = RobotControlUI()
    window.show()
    sys.exit(app.exec_())
