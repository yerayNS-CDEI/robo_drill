from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, RegisterEventHandler, TimerAction, GroupAction
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import ComposableNodeContainer, Node, PushRosNamespace
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sick",
            default_value="true"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "dome",
            default_value="true"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "oak",
            default_value="true",
            description="Whether to launch DepthAI OAK-D camera driver on real hardware"
        )
    )

    # Initialize Arguments
    launch_dome = LaunchConfiguration("dome")
    launch_sick = LaunchConfiguration("sick")
    launch_oak = LaunchConfiguration("oak")

    # Include the Ouster launch file if Ouster is detected
    dome_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([FindPackageShare('ouster_ros'), 'launch', 'dome_driver.launch.py'])
            ]),
            launch_arguments={'viz': 'false'}.items(),
        condition=IfCondition(launch_dome)
        )

    # NOTE: resolve these lazily via FindPackageShare (a substitution) rather than
    # get_package_share_directory(). The substitution is only evaluated when the
    # SICK nodes actually execute, which is gated by IfCondition(launch_sick) below.
    # This keeps the launch from crashing at build time when sick_scan_xd isn't
    # installed (e.g. simulation, or sick:=false) instead of aborting everything.
    rear_launch_file = PathJoinSubstitution([FindPackageShare('sick_scan_xd'), 'launch', 'sick_multiscan_rear.launch'])
    front_launch_file = PathJoinSubstitution([FindPackageShare('sick_scan_xd'), 'launch', 'sick_multiscan_front.launch'])
    
    
    # First SICK multiScan (192.168.1.60) - REAR
    sick_node_rear = GroupAction([
        PushRosNamespace('rear'),
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_rear',
            output='screen',
            arguments=[rear_launch_file, '--ros-args', '--log-level', 'warn'],
            remappings=[
            ('cloud_all_fields_fullframe', 'points'),  # Remap the topic name!
        ],
            condition=IfCondition(launch_sick)
        )
    ])
    
    # Second SICK multiScan (192.168.1.61) - FRONT
    sick_node_front = GroupAction([
        PushRosNamespace('front'),
        Node(
            package='sick_scan_xd',
            executable='sick_generic_caller',
            name='sick_front',
            output='screen',
            arguments=[front_launch_file, '--ros-args', '--log-level', 'warn'],
            remappings=[
            ('cloud_all_fields_fullframe', 'points'),  # Remap the topic name!
        ],
            condition=IfCondition(launch_sick)
        )
    ])   
    oak_launch = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='depthai_ros_driver',
                executable='camera_node',
                name='oak',
                namespace='',
                parameters=[PathJoinSubstitution([
                    FindPackageShare('robo_drill'),
                    'config',
                    'oak_params.yaml'  # This has i_pipeline_type: "RGBD"
                ])],
                output='screen',
                condition=IfCondition(launch_oak)
            )
        ]
    )
    

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py'])
        ]),
        launch_arguments={
            'serial_no': "'035122250320'",   # D455 (front)
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '640x480x15',
            'depth_module.depth_profile': '640x480x15',
        }.items(),
    )

    realsense_rear_launch = TimerAction(
        period=5.0,
        actions=[Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera_rear',
            parameters=[PathJoinSubstitution([
                FindPackageShare('robo_drill'), 'config', 'camera_rear_params.yaml'
            ])],
            output='screen',
        )]
    )
    
    # Connect camera_link as child of os_lidar (which is published by Ouster driver)
    # The Ouster driver publishes: os_sensor -> os_lidar
    # The URDF publishes: base_link -> turret_link -> os_sensor
    # This connects them: os_lidar (from driver) -> camera_link
    static_tf_camera = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0075', '-0.128', '0.015', 
                  '-0.747', '0.006', '-0.664', '0.005',
                  'os_sensor_mount', 'camera_link']
    )
    
    # Bridge URDF's os_sensor_mount to driver's os_sensor
    # This connects the robot's TF tree to the sensor's TF tree
    static_tf_os_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0',
                  'os_sensor_mount', 'os_sensor']
    )
    
    sensors = [
        dome_launch,
        sick_node_rear,
        sick_node_front,
        realsense_launch,
        realsense_rear_launch,
        static_tf_os_bridge,
        # static_tf_camera,
        # oak_launch,
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments + sensors)