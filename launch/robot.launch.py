# Copyright 2020 ros2_control Development Team
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

from launch import LaunchDescription
from launch.actions import RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from launch.conditions import UnlessCondition, IfCondition


def generate_launch_description():

    # setting this id will serve two purposes:
    # 1) identifying which robot is in use: red, green or yellow
    # 2) restricting communication between the selected robot and the control computer only

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    # Check if ros_domain_id is not in range 1-19
    if ros_domain_id is None or int(ros_domain_id) not in range(1, 20):  # Check if it is a string within range [1, 19]
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}. \n"
            "  Please set ROS_DOMAIN_ID using one of the following commands:\n"
            "  - set_moby_model GREEN\n"
            "  - set_moby_model RED\n"
            "  - export ROS_DOMAIN_ID=<value>"
        )

    package_path = FindPackageShare("robo_drill")
    params_file = PathJoinSubstitution([package_path, 'config/diffdrive_controllers.yaml'])

    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
        )
    )
    
    declared_arguments.append(
        DeclareLaunchArgument(
            "mode",
            default_value="full",
            description="Launch mode full|base",
            choices=['full', 'base'],
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_from_controller",
            default_value="false",
            description="Get odometry from omni controller.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="diff",
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
    
    # Initialize Arguments
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    mode = LaunchConfiguration("mode")
    odom_from_controller = LaunchConfiguration("odom_from_controller")
    controller_type = LaunchConfiguration("controller_type")
    publish_controller_odom_tf = LaunchConfiguration("publish_controller_odom_tf")
    controller_should_publish_tf = PythonExpression(
        ["'true' if '", publish_controller_odom_tf, "' == 'true' else '", odom_tf_from_controller, "'"]
    )
    controller_should_publish_odom = PythonExpression(
        ["'true' if '", publish_controller_odom_tf, "' == 'true' else '", odom_from_controller, "'"]
    )
    
    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [package_path, "robo_drill_description/description", "robo_drill.urdf.xacro"]
            ),
            " ",
            "use_mock_hardware:=false",
            " ",
            "simulation:=false",
            " ",
            "ros_domain_id:=", ros_domain_id,
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'enable_odom_tf': odom_tf_from_controller,      # diff drive controller provides odom->base_link tf
        'should_publish_tf': controller_should_publish_tf,   # omni drive controller provides odom->base_link tf
        'should_publish_odom': controller_should_publish_odom,    # omni drive controller provides odometry
        }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True)

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, configured_params],
        output="both",
        remappings=[
            ("/diffbot_base_controller/odom", "controller/odometry"),
            ("/sim_controller/odom", "controller/odometry"),
        ],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'base'"])
        )
    )
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'base'"])
        )
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    robot_controller_spawner_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "turret_controller", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'diff'"]))
    )

    robot_controller_spawner_omni = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_controller", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'omni'"]))
    )

    gantry_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_position_controller", "--controller-manager", "controller_manager"],
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawner_diff = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner_diff],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )
    delay_robot_controller_spawner_diff_full_mode = TimerAction(
        period=8.0,
        actions=[robot_controller_spawner_diff],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    delay_robot_controller_spawner_omni = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner_omni],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )
    
    delay_robot_controller_spawner_omni_full_mode = TimerAction(
        period=8.0,
        actions=[robot_controller_spawner_omni],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    # Delay gantry controller spawner after joint_state_broadcaster
    delay_gantry_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gantry_controller_spawner],
        ),
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    # Delayed gantry controller for full mode
    delayed_gantry_controller_spawner_full_mode = TimerAction(
        period=8.0,
        actions=[gantry_controller_spawner],
        condition=UnlessCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        delay_robot_controller_spawner_diff,
        delay_robot_controller_spawner_omni,

        delay_robot_controller_spawner_diff_full_mode,
        delay_robot_controller_spawner_omni_full_mode,
        delay_gantry_controller_spawner,
        delayed_gantry_controller_spawner_full_mode
    ]

    return LaunchDescription(declared_arguments+ nodes)
