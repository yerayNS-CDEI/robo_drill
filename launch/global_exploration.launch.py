import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

from launch.actions import RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit

def generate_launch_description():
    # --- Arguments to pass through CLI ---
    sim            = LaunchConfiguration('sim')
    world          = LaunchConfiguration('world')
    use_sim_time   = LaunchConfiguration('use_sim_time')
    # NOTE: this is intentionally NOT named 'params_file'. A LaunchConfiguration named
    # 'params_file' would leak down the include chain (mapping_3d -> pokeye -> platform ->
    # sensors -> ouster dome_driver), overriding dome_driver's default dome_params.yaml and
    # leaving sensor_hostname unset, which makes os_driver fail to configure and tears down
    # the whole launch. Launching mapping_3d.launch.py directly works precisely because it
    # never declares 'params_file'. Keep this name scoped to Nav2 only.
    nav2_params_file = LaunchConfiguration('nav2_params_file')
    log_level      = LaunchConfiguration('log_level')
    # Loose goal tolerances during frontier exploration: the robot only needs to get
    # "near" each frontier to keep mapping moving, not settle precisely on it. Localization
    # keeps the strict navigation_launch.py defaults. Override at the CLI if needed.
    #
    # NOTE: xy_goal_tolerance MUST stay below explore's min_goal_distance (0.7 in
    # explore_params.yaml). The within-footprint bootstrap nudges to a goal only
    # min_goal_distance + 0.1 = 0.8 m away; if the tolerance meets/exceeds that, Nav2
    # reports the goal reached without the robot moving and exploration spins in place
    # ("Goal reached -> nudging forward -> New GOAL" loop). 0.5 leaves real motion margin.
    xy_goal_tolerance  = LaunchConfiguration('xy_goal_tolerance')
    yaw_goal_tolerance = LaunchConfiguration('yaw_goal_tolerance')

    declare_args = [
        DeclareLaunchArgument('sim',           default_value='true'),
        DeclareLaunchArgument('use_sim_time',  default_value='true'),
        DeclareLaunchArgument('log_level',     default_value='warn'),
        DeclareLaunchArgument('xy_goal_tolerance',  default_value='0.5'),
        DeclareLaunchArgument('yaw_goal_tolerance', default_value='3.14'),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robo_drill'),
                'config',
                'nav2_params_omni.yaml'
            ])
        ),
    ]

    # --- File routes ---
    navi_share = FindPackageShare('robo_drill')

    # --- Include: mapping 3D (mapping_3d.launch.py) ---
    mapping_3d = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([navi_share, 'launch', 'mapping_3d.launch.py'])
        ]),
        launch_arguments={
            'sim':  sim,
            'log_level': log_level,
            'controller_type': 'omni',
            'mode': 'base',
        }.items()
    )

    # --- Include: Nav2 (navigation_launch.py) with params_file ---
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([navi_share, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file':  nav2_params_file,
            'log_level':    log_level,
            'controller_type': 'omni',
            'xy_goal_tolerance':  xy_goal_tolerance,
            'yaw_goal_tolerance': yaw_goal_tolerance,
        }.items()
    )

    # --- Nodes: find_frontiers and exploration_closest_frontier ---
    find_frontiers = Node(
        package='robo_drill',
        executable='find_frontiers',
        name='find_frontiers',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
    )

    exploration = Node(
        package='robo_drill',
        executable='explore',
        name='explore',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--params-file', PathJoinSubstitution([
                FindPackageShare('robo_drill'),
                'config',
                'explore_params.yaml'
            ])],
    )

    # Delaying Nav2 and exploration nodes
    delayed_nav2 = TimerAction(period=3.0, actions=[nav2])
    delayed_exploration = TimerAction(period=5.0, actions=[exploration])

    return LaunchDescription(
        declare_args + [
            mapping_3d,
            delayed_nav2,
            delayed_exploration,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=exploration,
                    on_exit=[Shutdown(reason='Exploration finished')]
                )
            )
        ]        
    )
