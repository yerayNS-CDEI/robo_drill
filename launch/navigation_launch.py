# Copyright (c) 2018 Intel Corporation
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
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch_ros.actions import LoadComposableNodes
from launch_ros.actions import Node
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('robo_drill')
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    controller_type = LaunchConfiguration('controller_type')
    use_dynamic_footprint = LaunchConfiguration('use_dynamic_footprint')
    custom_recovery_behaviors = LaunchConfiguration('custom_recovery_behaviors')
    xy_goal_tolerance = LaunchConfiguration('xy_goal_tolerance')
    yaw_goal_tolerance = LaunchConfiguration('yaw_goal_tolerance')

    lifecycle_nodes = ['controller_server',
                       'smoother_server',
                       'planner_server',
                       'behavior_server',
                       'bt_navigator',
                       'waypoint_follower',
                       'velocity_smoother']

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    # TODO(orduno) Substitute with `PushNodeRemapping`
    #              https://github.com/ros2/launch_ros/issues/56
    remappings = [('/tf', 'tf'),
                  ('/tf_static', 'tf_static')]
    
    # Conditionally select nav2 params file regarding selected controller type
    package_dir = get_package_share_directory("robo_drill")
    diff_nav2_params = PathJoinSubstitution([package_dir, "config", "nav2_params.yaml"])
    omni_nav2_params = PathJoinSubstitution([package_dir, "config", "nav2_params_omni.yaml"])
    nav2_params_file = PythonExpression([
        "'",
        omni_nav2_params,
        "' if '",
        controller_type,
        "' == 'omni' else '",
        diff_nav2_params,
        "'"
    ])
    dynamic_footprint_base_frame = PythonExpression([
        "'turret_footprint' if '",
        controller_type,
        "' == 'omni' else 'base_footprint'"
    ])

    # Recovery behavior tree selection. When custom_recovery_behaviors is 'true'
    # the bt_navigator uses our tree (RetractArmFromObstacle + BackOutFromObstacle
    # escapes); when 'false' it falls back to the stock nav2 tree (spin/backup/wait
    # only), so the custom arm-retract/back-out recoveries are disabled - e.g.
    # while the wall-scanning FSM is in control.
    custom_recovery_bt = os.path.join(
        package_dir, "config", "behavior_trees",
        "navigate_to_pose_w_backout_recovery.xml")
    stock_recovery_bt = os.path.join(
        get_package_share_directory("nav2_bt_navigator"), "behavior_trees",
        "navigate_to_pose_w_replanning_and_recovery.xml")
    nav_to_pose_bt_xml = PythonExpression([
        "'", custom_recovery_bt, "' if '",
        custom_recovery_behaviors, "' == 'true' else '", stock_recovery_bt, "'"])

    # Create our own temporary YAML files that include substitutions.
    # The goal-tolerance rewrites use full dotted paths so they only touch the
    # controller goal checker (general_goal_checker in the diff params,
    # goal_checker in the omni params) and NOT DWB's FollowPath.xy_goal_tolerance.
    # Only the path matching the selected params file exists; the other is ignored.
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'autostart': autostart,
        'default_nav_to_pose_bt_xml': nav_to_pose_bt_xml,
        'controller_server.ros__parameters.general_goal_checker.xy_goal_tolerance': xy_goal_tolerance,
        'controller_server.ros__parameters.general_goal_checker.yaw_goal_tolerance': yaw_goal_tolerance,
        'controller_server.ros__parameters.goal_checker.xy_goal_tolerance': xy_goal_tolerance,
        'controller_server.ros__parameters.goal_checker.yaw_goal_tolerance': yaw_goal_tolerance}

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
    allow_substs=True)

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true')

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_dir, 'config', 'nav2_params.yaml'), #! not working
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Use composed bringup if True')

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='the name of conatiner that nodes will load in if use composition')

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn', default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.')

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='warn',
        description='log level')
    
    declare_controller_type_cmd = DeclareLaunchArgument(
        'controller_type',
        default_value='diff',
        description='Type of controller to use (diff or omni)')

    declare_use_dynamic_footprint_cmd = DeclareLaunchArgument(
        'use_dynamic_footprint',
        default_value='false',
        description='Launch dynamic footprint publisher for arm-aware costmap footprint updates')

    declare_custom_recovery_behaviors_cmd = DeclareLaunchArgument(
        'custom_recovery_behaviors',
        default_value='true',
        description='Enable the custom arm-retract + back-out recovery behaviors '
                    '(true). Set false to use the stock nav2 recovery tree only, '
                    'e.g. while the wall-scanning FSM is in control.')

    # Resolved into the RetractArmFromObstacle recovery's `planner_backend` param
    # via RewrittenYaml allow_substs ($(var planner_backend) in nav2_params_omni.yaml),
    # so arm-retract routes through the matching arm planner.
    declare_planner_backend_cmd = DeclareLaunchArgument(
        'planner_backend',
        default_value='moveit',
        description='Arm planner backend (moveit or legacy); selects the arm-retract recovery path')

    # Goal tolerances are rewritten into the controller_server goal_checker via the
    # RewrittenYaml param_rewrites above. Defaults preserve the strict localization
    # values; callers can pass looser values if needed. The yaw default is
    # controller-aware to match each params file (diff 0.35 vs omni 0.25).
    declare_xy_goal_tolerance_cmd = DeclareLaunchArgument(
        'xy_goal_tolerance',
        default_value='0.25',
        description='XY goal tolerance (m) for the controller goal checker')

    declare_yaw_goal_tolerance_cmd = DeclareLaunchArgument(
        'yaw_goal_tolerance',
        default_value=PythonExpression(
            ["'0.35' if '", controller_type, "' == 'diff' else '0.25'"]),
        description='Yaw goal tolerance (rad) for the controller goal checker')

    dynamic_footprint_node = Node(
        package='robo_drill',
        executable='dynamic_footprint_publisher.py',
        name='dynamic_footprint_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_radius': 0.5,
            'local_footprint_topic': 'local_costmap/footprint',
            'global_footprint_topic': 'global_costmap/footprint',
            'joint_states_topic': '/joint_states',
            'publish_rate_hz': 20.0,
            'joint_state_timeout_s': 1.0,
            'enable_arm_expansion': True,
            'use_tf_for_arm_tip': True,
            'robot_base_frame': dynamic_footprint_base_frame,
            'arm_tool_frame': 'arm_tool0',
            'tf_timeout_s': 0.10,
            'arm_direction_yaw_offset': 0.0,
            'arm_shoulder_pan_joint': 'arm_shoulder_pan_joint',
            'arm_shoulder_lift_joint': 'arm_shoulder_lift_joint',
            'arm_elbow_joint': 'arm_elbow_joint',
            'arm_mount_x': 0.0,
            'arm_mount_y': 0.0,
            'arm_mount_yaw': -2.3562,
            'upper_arm_length': 0.613,
            'forearm_length': 0.572,
            'tool_padding': 0.10,
            'arm_tip_radius': 0.20,
            'max_arm_reach': 1.8,
            'reach_alpha': 1.0,
        }],
        condition=IfCondition(
            PythonExpression([
                "'",
                use_dynamic_footprint,
                "' == 'true'",
            ])
        )
    )

    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            Node(
                package='nav2_controller',
                executable='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings +
                        [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}]),
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(use_composition),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_controller',
                plugin='nav2_controller::ControllerServer',
                name='controller_server',
                parameters=[configured_params],
                remappings=remappings + [('cmd_vel', 'cmd_vel_nav')]),
            ComposableNode(
                package='nav2_smoother',
                plugin='nav2_smoother::SmootherServer',
                name='smoother_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_planner',
                plugin='nav2_planner::PlannerServer',
                name='planner_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_behaviors',
                plugin='behavior_server::BehaviorServer',
                name='behavior_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_bt_navigator',
                plugin='nav2_bt_navigator::BtNavigator',
                name='bt_navigator',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_waypoint_follower',
                plugin='nav2_waypoint_follower::WaypointFollower',
                name='waypoint_follower',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_velocity_smoother',
                plugin='nav2_velocity_smoother::VelocitySmoother',
                name='velocity_smoother',
                parameters=[configured_params],
                remappings=remappings +
                           [('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')]),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_navigation',
                parameters=[{'use_sim_time': use_sim_time,
                             'autostart': autostart,
                             'node_names': lifecycle_nodes}]),
        ],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)
    ld.add_action(declare_use_respawn_cmd)
    ld.add_action(declare_log_level_cmd)
    ld.add_action(declare_controller_type_cmd)
    ld.add_action(declare_use_dynamic_footprint_cmd)
    ld.add_action(declare_planner_backend_cmd)
    ld.add_action(declare_custom_recovery_behaviors_cmd)
    ld.add_action(declare_xy_goal_tolerance_cmd)
    ld.add_action(declare_yaw_goal_tolerance_cmd)
    # Add the actions to launch all of the navigation nodes
    ld.add_action(dynamic_footprint_node)
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld
