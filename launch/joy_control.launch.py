import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition

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
            "viz",
            default_value="false",
            description="launch rviz for visualization",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="true",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log Level",
        )
    )

    package_path = FindPackageShare("robo_drill")
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/navigation.rviz'])
    # Launch arguments
    simulation_mode = LaunchConfiguration('sim')
    viz = LaunchConfiguration('viz')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    log_level = LaunchConfiguration("log_level")


    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('robo_drill'), 'launch', 'platform.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                          'odom_tf_from_controller': odom_tf_from_controller}.items()
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
        condition=IfCondition(viz)
    )

    nodes = [
        platform_launch,
        rviz_node
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments+nodes)