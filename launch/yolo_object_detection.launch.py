from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
import platform

def generate_launch_description():
    # Get conda environment paths
    conda_prefix = os.path.expanduser('~/miniconda3/envs/yolo_discover')
    
    # Path to DISCOVER-YOLO-Detection for models
    discover_path = os.path.expanduser('~/DISCOVER-YOLO-Detection')
    
    arch = platform.machine()
    if arch == 'x86_64':
        lib_path = '/usr/lib/x86_64-linux-gnu'
    elif arch in ['aarch64', 'arm64']:
        lib_path = '/usr/lib/aarch64-linux-gnu'
    else:
        lib_path = '/usr/lib'
    
    return LaunchDescription([
        # Set environment variables for conda and ROS2 compatibility
        SetEnvironmentVariable(
            name='LD_PRELOAD',
            value=f'{lib_path}/libstdc++.so.6'
        ),
        # Append conda libs AFTER the existing (ROS-sourced) LD_LIBRARY_PATH so
        # rclpy can still find /opt/ros/humble/lib (librcl_action.so, etc.).
        SetEnvironmentVariable(
            name='LD_LIBRARY_PATH',
            value=[EnvironmentVariable('LD_LIBRARY_PATH', default_value=''),
                   f':{lib_path}:{conda_prefix}/lib']
        ),
        
        # Launch arguments
        DeclareLaunchArgument(
            'model_path',
            default_value=f'{discover_path}/models/best.pt',
            description='Path to YOLO model file'
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/controller/odometry',
            description='Odometry topic name'
        ),
        DeclareLaunchArgument(
            'csv_output_dir',
            default_value='rgb_detections',
            description='Directory for CSV output. Relative paths are created inside the robo_drill package share directory.'
        ),
        DeclareLaunchArgument(
            'display_window',
            default_value='true',
            description='Show display window'
        ),
        DeclareLaunchArgument(
            'camera_optical_frame',
            default_value='oak_rgb_camera_optical_frame',
            description='TF frame of the OAK RGB optical sensor published by the robot URDF/TF tree'
        ),
        DeclareLaunchArgument(
            'ee_frame',
            default_value='tool0',
            description='Deprecated legacy argument; ignored because the camera TF now comes from the URDF'
        ),
        DeclareLaunchArgument(
            'map_frame',
            default_value='map',
            description='Target frame for the 3D door/window coordinates'
        ),

        # YOLO Detection Node
        Node(
            package='robo_drill',
            executable='yolo_object_detection.py',
            name='yolo_detection_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'csv_output_dir': LaunchConfiguration('csv_output_dir'),
                'display_window': LaunchConfiguration('display_window'),
                'camera_optical_frame': LaunchConfiguration('camera_optical_frame'),
                'ee_frame': LaunchConfiguration('ee_frame'),
                'map_frame': LaunchConfiguration('map_frame'),
                'preview_size': [1280, 720],
                'fps': 15,  # USB2-safe with the added stereo-depth stream
                'conf_threshold': 0.6,
                'target_classes': ['door', 'window'],
                'max_range_m': 8.0,
                'min_plane_points': 50,
                'force_usb2': False,
            }],
            # Use conda environment's Python
            prefix=[f'{conda_prefix}/bin/python3 -u']
        ),
    ])
