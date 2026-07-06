#!/usr/bin/env python3
"""
Hybrid Simulation Launch File

This launch file enables simulation with:
- Gazebo Ignition for the mobile base (environment, mapping, sensors)
- URSim for the UR arm control

The key challenge is that Gazebo and URSim have separate TF trees. This launch
file bridges them using a static transform publisher.

TF Tree Structure:
- Gazebo provides: odom -> base_footprint -> base_link -> ... -> gantry_z_link
- URSim provides:  arm_base_link -> arm_shoulder_link -> ... -> arm_tool0
- This file adds:  gantry_z_link -> world (static transform)

Usage:
    ros2 launch robo_drill hybrid_simulation.launch.py

This connects the end effector to the map frame, allowing you to know the
arm position relative to the environment.
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    GroupAction,
    OpaqueFunction,
)
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch.conditions import IfCondition
from launch_ros.actions import Node, SetParameter
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_arm_stack(context, *args, **kwargs):
    """Launch arm stack with evaluated parameters."""
    # Get the original_mode from kwargs (passed from generate_launch_description)
    original_mode = kwargs.get('original_mode', 'full')
    planner_backend = context.launch_configurations.get('planner_backend', 'legacy')
    moveit_planning_pipeline = context.launch_configurations.get(
        'moveit_planning_pipeline', 'pilz_industrial_motion_planner'
    )
    moveit_pose_planner_id = context.launch_configurations.get(
        'moveit_pose_planner_id', 'PTP'
    )
    moveit_joint_planner_id = context.launch_configurations.get(
        'moveit_joint_planner_id', 'PTP'
    )
    robot_ip = context.launch_configurations.get('robot_ip', '192.168.56.101')
    tf_prefix = context.launch_configurations.get('tf_prefix', 'arm_')
    ur_type = context.launch_configurations.get('ur_type', 'ur10e')
    launch_rviz = context.launch_configurations.get('launch_rviz', 'true')
    rviz_config_file = context.launch_configurations.get(
        'rviz_config_file',
        '',
    )
    moveit_joint_states_topic = context.launch_configurations.get(
        'moveit_joint_states_topic',
        '',
    )
    
    arm_package_path = FindPackageShare("arm_control")
    
    arm_launch_group = GroupAction([
        SetParameter(name='use_sim_time', value=True),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([arm_package_path, 'launch', 'arm.launch.py'])
            ]),
            launch_arguments={
                'sim': 'false',  # Not simulated in Gazebo - connecting to URSim
                'mode': 'arm',   # Keep control stack arm-only to avoid duplicating the base TF tree
                'moveit_mode': original_mode,  # Pass through the requested mode for MoveIt planning (full/arm)
                'moveit_use_sim_time': 'true',  # Keep MoveIt on Gazebo clock in hybrid localization
                'planner_backend': planner_backend,
                'moveit_planning_pipeline': moveit_planning_pipeline,
                'moveit_pose_planner_id': moveit_pose_planner_id,
                'moveit_joint_planner_id': moveit_joint_planner_id,
                'robot_ip': robot_ip,
                'tf_prefix': tf_prefix,
                'ur_type': ur_type,
                'launch_rviz': launch_rviz,
                'rviz_config_file': rviz_config_file,
                'moveit_joint_states_topic': moveit_joint_states_topic,
                'publish_controller_odom_tf': context.launch_configurations.get(
                    'publish_controller_odom_tf', 'false'
                ),
                'controllers_file': 'arm_only_controllers.yaml',  # Arm-only controllers
                'namespace_arm': 'arm',  # Namespace to avoid controller_manager conflicts
            }.items(),
        ),
    ])
    
    return [arm_launch_group]


def generate_launch_description():
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id is None or int(ros_domain_id) not in range(1, 20):
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}"
        )

    base_package_path = FindPackageShare("robo_drill")

    declared_arguments = []

    # World selection for Gazebo
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="castelldefels_indoors_empty",
            description="World file for Gazebo simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run simulation without the Gazebo GUI.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [FindPackageShare("robo_drill"), "rviz", "navigation.rviz"]
            ),
            description="RViz config file to use for the MoveIt-capable RViz instance",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="omni",
            description="Which base controller to use (diffdrive or omni)",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "publish_controller_odom_tf",
            default_value="false",
            description="Override controller YAMLs so the controller publishes odom and TF.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="full",
            description="Requested robot mode for localization workflows",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "planner_backend",
            default_value="legacy",
            description="Planner backend to use: legacy or moveit",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_planning_pipeline",
            default_value="pilz_industrial_motion_planner",
            description="MoveIt planning pipeline for the arm stack",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_pose_planner_id",
            default_value="PTP",
            description="MoveIt planner id for pose goals",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_planner_id",
            default_value="PTP",
            description="MoveIt planner id for joint goals",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_states_topic",
            default_value="",
            description="Optional joint_states topic override for MoveIt",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.56.101",
            description="IP address of URSim or real UR robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "tf_prefix",
            default_value="arm_",
            description="TF prefix for the arm",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur10e",
            description="UR robot type",
        )
    )

    # Create OpaqueFunction to capture all launch logic
    def setup_launches(context, *args, **kwargs):
        # Capture the original mode value BEFORE any included launches modify it
        original_mode = context.launch_configurations.get('mode', 'full')
        world_name = context.launch_configurations.get('world', 'castelldefels_indoors_empty')
        controller_type = context.launch_configurations.get('controller_type', 'omni')
        
        print(f"[hybrid_simulation.launch.py DEBUG] Captured original mode: '{original_mode}'")
        
        # =========================================================================
        # 1. Launch Gazebo simulation for the BASE (mode=base)
        #    This uses robo_drill.urdf.xacro and publishes base TF tree
        # =========================================================================
        base_simulation_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([base_package_path, 'launch', 'platform.launch.py'])
            ]),
            launch_arguments={
                'sim': 'true',
                'world': world_name,
                'headless': context.launch_configurations.get('headless', 'false'),
                'mode': 'base',  # Base only - will use robo_drill.urdf.xacro
                'controller_type': controller_type,
                'publish_controller_odom_tf': context.launch_configurations.get(
                    'publish_controller_odom_tf', 'false'
                ),
            }.items(),
        )
        
        planner_backend = context.launch_configurations.get('planner_backend', 'legacy')
        moveit_joint_states_topic = context.launch_configurations.get(
            'moveit_joint_states_topic', ''
        )

        joint_state_merger = Node(
            package='arm_control',
            executable='joint_state_merger_node',
            name='moveit_joint_state_merger',
            output='screen',
            parameters=[
                {
                    'input_topics': ['/joint_states', '/arm/joint_states'],
                    'output_topic': '/moveit_joint_states',
                    'publish_rate': 30.0,
                    'use_sim_time': True,
                }
            ],
            condition=IfCondition(
                PythonExpression(
                    [
                        "'",
                        planner_backend,
                        "' == 'moveit' and '",
                        original_mode,
                        "' == 'full'",
                    ]
                )
            ),
        )

        if planner_backend == 'moveit' and original_mode == 'full' and not moveit_joint_states_topic:
            context.launch_configurations['moveit_joint_states_topic'] = '/moveit_joint_states'

        # Create arm launch with captured original_mode
        arm_launch_function = OpaqueFunction(
            function=launch_arm_stack,
            kwargs={'original_mode': original_mode}
        )
        
        # =========================================================================
        # 2. Static Transform Publishers with sim_time
        #    CRITICAL: All TF publishers must use sim_time when use_sim_time:=true
        #    Without this, static transforms will have timestamp 0 while dynamic
        #    transforms use sim_time (~260s), causing TF lookup failures.
        #
        #    This connects gantry_z_link (from base) to world (arm's root frame)
        #
        #    From mobile_manipulator.urdf.xacro, the arm mounting transform is:
        #        xyz="0.0 0.0 0.73" rpy="0.0 0.0 -2.3562"
        #
        #    Note: The arm URDF uses 'world' as its root frame (UR convention).
        #    The arm's robot_state_publisher creates: world → arm_base_link → ...
        #    We must connect to 'world' to avoid TF conflicts.
        #
        #    Transform: gantry_z_link is parent, world is child
        #    xyz: 0.0 0.0 0.73 (arm mounting is 0.73m above gantry_z_link)
        #    rpy: 0.0 0.0 -2.3562 (arm is rotated -135 degrees around Z)
        # =========================================================================
        static_tf_group = GroupAction([
        SetParameter(name='use_sim_time', value=True),

        # The static_transform_publisher expects: x y z yaw pitch roll parent child
        # Note: static_transform_publisher uses yaw-pitch-roll order (Z-Y-X), not roll-pitch-yaw
        # rpy in URDF = (roll, pitch, yaw) = (0, 0, -2.3562)
        # For static_transform_publisher arguments: x y z yaw pitch roll
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_arm_tf_bridge',
            arguments=[
                '--x', '0.0',
                '--y', '0.0',
                '--z', '0.73',
                '--roll', '0.0',
                '--pitch', '0.0',
                '--yaw', '-2.3562',
                '--frame-id', 'gantry_z_link',
                '--child-frame-id', 'world',  # Connect to arm URDF root frame
            ],
            output='screen',
        ),
        ])
        
        return [base_simulation_launch, joint_state_merger, arm_launch_function, static_tf_group]
    
    return LaunchDescription(
        declared_arguments + [OpaqueFunction(function=setup_launches)]
    )
