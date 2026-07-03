import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

def generate_launch_description():
    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    # Check if ros_domain_id is not in range 1-19
    if int(ros_domain_id) not in range(1, 20):  # Check if it is a string within range [1, 19]
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}. \n"
            "  Please set ROS_DOMAIN_ID using one of the following commands:\n"
            "  - set_moby_model GREEN\n"
            "  - set_moby_model RED\n"
            "  - export ROS_DOMAIN_ID=<value>"
        )

    # get paths
    package_path = FindPackageShare("robo_drill")
    slam_mapping_params_file = PathJoinSubstitution([package_path, 'config/mapper_params_online_async.yaml'])
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/mapping_2D.rviz'])

    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="simulation mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="warehouse",
            description="world file for simulation",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "lidar",
            default_value="sick",
            description="Which Lidar sensor to use (ouster or sick)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log Level",
        )
    )

    simulation_mode = LaunchConfiguration('sim')
    world_name = LaunchConfiguration('world')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    lidar_name = LaunchConfiguration('lidar')
    log_level = LaunchConfiguration('log_level')

    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('robo_drill'), 'launch', 'platform.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                          'world': world_name,
                          'odom_tf_from_controller':odom_tf_from_controller,
                          'lidar': lidar_name}.items()
    )

    # launch 2D SLAM mapping
    slam_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'online_async.launch.py'])
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'slam_params_file': slam_mapping_params_file }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
    )

    nodes = [
        platform_launch,
        slam_mapping_node,
        rviz_node,
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments+nodes)
