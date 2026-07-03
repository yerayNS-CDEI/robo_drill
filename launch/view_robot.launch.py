# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_package",
            default_value="robo_drill",
            description="Description package with robot URDF/xacro files. Usually the argument \
        is not set, it enables use of a custom description.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "description_file",
            default_value="robo_drill.urdf.xacro",
            description="URDF/XACRO description file with the robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log Level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="empty_world",
            description="path to world file",
        )
    )

    package_path = FindPackageShare("robo_drill")

    # Initialize Arguments
    description_file = LaunchConfiguration("description_file")
    world_name = LaunchConfiguration('world')
    world_dir = PathJoinSubstitution([package_path, 'worlds', world_name])
    world_file_name = PathJoinSubstitution([world_dir, PythonExpression(["'", world_name, "' + '.sdf'"])])
    log_level = LaunchConfiguration('log_level')

    # Get URDF via xacro
    robot_description_content = Command(
        # The idea is to run the whole thing as a command you would type in the terminal
        [FindExecutable(name="xacro"), # get path to the executable /opt/ros/humble/bin/xacro
        " ", # add space
        PathJoinSubstitution([package_path, "robo_drill_description/description", description_file])] # get path to the robot description xacro file
    )

    robot_description = {"robot_description": robot_description_content}

    rviz_config_file = PathJoinSubstitution(
        [package_path, "rviz", "robot_view.rviz"]
    )

    # Gazebo Sim (Ignition)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r ', world_file_name],
        }.items(),
    )

    # Bridge /clock from Gz to ROS
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        ],
        output='screen',
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-name', 'robot',
                    '-x', '-2.0',
                    '-y', '-0.5',
                   '-z', '0.21'],
        output='screen'
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
    )

    # Set IGN_GAZEBO_RESOURCE_PATH so Gz Sim can resolve package:// mesh URIs
    pkg_share_dir = get_package_share_directory('robo_drill')
    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_share_dir),
    )

    nodes = [
        set_gz_resource_path,
        gz_sim,
        gz_bridge,
        spawn_robot,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
