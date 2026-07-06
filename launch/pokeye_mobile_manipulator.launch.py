#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction, ExecuteProcess, IncludeLaunchDescription
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, AndSubstitution, NotSubstitution, PythonExpression
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from nav2_common.launch import RewrittenYaml
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id is None or int(ros_domain_id) not in range(1, 20):
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}"
        )

    base_package_path = FindPackageShare("robo_drill")

    declared_arguments = []
    
    # General arguments
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value='false',
            description="Use simulation mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "hybrid_sim",
            default_value='false',
            description="Use URSim for the arm while keeping the full stack launch active",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="full",
            description="Launch mode full|base|arm",
            choices=['full', 'base', 'arm'],
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="castelldefels_indoors_empty",
            description="world file for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz", 
            default_value="true", 
            description="Launch RViz?")
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
            "rviz_config_file",
            default_value=PathJoinSubstitution(
                [base_package_path, "rviz", "view_robot.rviz"]
            ),
            description="RViz config file for the full robot bringup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ethercat_interface",
            default_value="eno1",
            description="Network interface used by the Navi Wall EtherCAT master",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value=PythonExpression(
                ["'192.168.56.101' if '", LaunchConfiguration("hybrid_sim"), "' == 'true' else '192.168.1.102'"]
            ),
            description="IP address for the UR robot or URSim instance",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="omni",
            description="Which controller to launch (diffdrive or omni)",
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
            "realsense_color_profile",
            default_value="640x480x15",
            description="RealSense RGB stream profile as widthxheightxfps",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_depth_profile",
            default_value="640x480x15",
            description="RealSense depth stream profile as widthxheightxfps",
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

    # Initialize Arguments    
    simulation_mode = LaunchConfiguration('sim')
    hybrid_sim = LaunchConfiguration("hybrid_sim")
    mode = LaunchConfiguration("mode")
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    launch_rviz = LaunchConfiguration("launch_rviz")
    ethercat_interface = LaunchConfiguration("ethercat_interface")
    robot_ip = LaunchConfiguration("robot_ip")
    rviz_config_file = LaunchConfiguration("rviz_config_file")
    controller_type = LaunchConfiguration("controller_type")
    publish_controller_odom_tf = LaunchConfiguration("publish_controller_odom_tf")
    realsense_color_profile = LaunchConfiguration("realsense_color_profile")
    realsense_depth_profile = LaunchConfiguration("realsense_depth_profile")
    planner_backend = LaunchConfiguration("planner_backend")
    moveit_planning_pipeline = LaunchConfiguration("moveit_planning_pipeline")
    moveit_pose_planner_id = LaunchConfiguration("moveit_pose_planner_id")
    moveit_joint_planner_id = LaunchConfiguration("moveit_joint_planner_id")
    namespace_arm = ''


    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([base_package_path, 'launch', 'platform.launch.py'])
        ]),
        launch_arguments={
            'sim': simulation_mode,
            'world': world_name,
            'headless': headless,
            'mode': mode,
            'controller_type': controller_type,
            'publish_controller_odom_tf': publish_controller_odom_tf,
            'realsense_color_profile': realsense_color_profile,
            'realsense_depth_profile': realsense_depth_profile,
            }.items(),
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'arm'"])),
    )

    # Include pointcloud concatenate launch file
    pointcloud_concatenate_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([base_package_path, 'launch', 'pointcloud_concatenate.launch.py'])
        ]),
        condition=IfCondition(PythonExpression([
            "('", mode, "' != 'arm') and ('", simulation_mode, "' == 'false')"
        ])),
    )
    
    # The manipulator half of the robot: the gantry geometry + gantry controller.
    # This is the robo_drill-local replacement for arm_control/arm.launch.py; there
    # is no separate manipulator package for this robot.
    manipulator_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([base_package_path, 'launch', 'manipulator.launch.py'])
        ]),
        launch_arguments={
            'sim': simulation_mode,
            'controller_type': controller_type,
            'publish_controller_odom_tf': publish_controller_odom_tf,
            'ethercat_interface': ethercat_interface,
            'launch_rviz': launch_rviz,
            'rviz_config_file': rviz_config_file,
            }.items(),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'full'"])),
    )

    return LaunchDescription(declared_arguments + [platform_launch, pointcloud_concatenate_launch, manipulator_launch])
