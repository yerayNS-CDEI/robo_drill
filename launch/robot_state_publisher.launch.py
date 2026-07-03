import os

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from launch_ros.substitutions import FindPackageShare
import xacro


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
    # Check if we're told to use sim time
    use_sim_time = LaunchConfiguration('sim')
    lidar_name = LaunchConfiguration("lidar")


    # Process the URDF file
    # using a command substitution here to run the xacro executable on our robot description file
    # to get the urdf (xml format) from the xacro file as a string
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("robo_drill"), "robo_drill_description/description", "robo_drill.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=false",
            " ",
            "simulation:=false",
            " ",
            "ros_domain_id:=", ros_domain_id,
            " ",
            "lidar:=", lidar_name,
        ]
    )

    params = {'robot_description': robot_description_content, 'use_sim_time': use_sim_time}
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    # Launch!
    return LaunchDescription([
        DeclareLaunchArgument(
            'sim',
            default_value='false',
            description='Use sim time if true'),

        DeclareLaunchArgument(
            'lidar',
            default_value='sick',
            description='which lidar to use for urdf'),
        node_robot_state_publisher
    ])

