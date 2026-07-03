"""Persistent mapping + nav2 stack (no frontier exploration).

This is the long-lived half of the old ``global_exploration.launch.py``: it
brings up rtabmap 3D mapping and Nav2, but NOT the ``find_frontiers`` /
``explore`` nodes, and it does NOT register a shutdown handler. The exploration
nodes are launched separately (see ``exploration.launch.py``) so that when they
finish, this stack keeps running — the robot stays localized, mapping and
navigable for a second structured coverage (densification) pass.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim          = LaunchConfiguration('sim')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file  = LaunchConfiguration('params_file')
    log_level    = LaunchConfiguration('log_level')
    # Loose goal tolerances while mapping (matches global_exploration.launch.py);
    # localization keeps the strict navigation_launch.py defaults. Override at the CLI.
    # NOTE: keep xy_goal_tolerance below explore's min_goal_distance (0.7); see the
    # detailed comment in global_exploration.launch.py.
    xy_goal_tolerance  = LaunchConfiguration('xy_goal_tolerance')
    yaw_goal_tolerance = LaunchConfiguration('yaw_goal_tolerance')

    declare_args = [
        DeclareLaunchArgument('sim',          default_value='true'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level',    default_value='warn'),
        DeclareLaunchArgument('xy_goal_tolerance',  default_value='0.5'),
        DeclareLaunchArgument('yaw_goal_tolerance', default_value='3.14'),
        DeclareLaunchArgument(
            'params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('robo_drill'),
                'config',
                'nav2_params_omni.yaml'
            ])
        ),
    ]

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
            'params_file':  params_file,
            'log_level':    log_level,
            'controller_type': 'omni',
            'xy_goal_tolerance':  xy_goal_tolerance,
            'yaw_goal_tolerance': yaw_goal_tolerance,
        }.items()
    )

    # Delay Nav2 so mapping_3d (TF + odom) is up first.
    delayed_nav2 = TimerAction(period=3.0, actions=[nav2])

    return LaunchDescription(declare_args + [mapping_3d, delayed_nav2])
