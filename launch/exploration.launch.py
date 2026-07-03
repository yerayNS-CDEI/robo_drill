"""Frontier-exploration nodes only (self-terminating).

This is the short-lived half of the old ``global_exploration.launch.py``: it
runs ``find_frontiers`` + ``explore`` against an ALREADY-RUNNING mapping + nav2
stack (see ``mapping_stack.launch.py``). When ``explore`` finishes (no frontiers
left) the ``OnProcessExit`` handler shuts THIS launch down, so the launch
process exits while the mapping stack keeps running. The CreateMap FSM state
detects exploration completion by polling this launch process's exit code.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level    = LaunchConfiguration('log_level')

    declare_args = [
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('log_level',    default_value='warn'),
    ]

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

    # Give find_frontiers a moment to start publishing before explore consumes.
    delayed_exploration = TimerAction(period=2.0, actions=[exploration])

    return LaunchDescription(
        declare_args + [
            find_frontiers,
            delayed_exploration,
            RegisterEventHandler(
                OnProcessExit(
                    target_action=exploration,
                    on_exit=[Shutdown(reason='Exploration finished')]
                )
            ),
        ]
    )
