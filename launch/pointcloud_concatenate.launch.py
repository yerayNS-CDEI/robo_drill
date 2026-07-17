import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    param_file_path = os.path.join(
        get_package_share_directory("robo_drill"),
        "config",
        "concatenate_params.yaml",
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("target_frame", default_value="front_sick_scan"),
            DeclareLaunchArgument("transform_timeout_sec", default_value="0.05"),
            DeclareLaunchArgument("fallback_to_latest_transform", default_value="true"),
            Node(
                package="robo_drill",
                executable="pointcloud_concatenate_node",
                name="pointcloud_merge",
                output="screen",
                parameters=[
                    param_file_path,
                    {
                        "target_frame": LaunchConfiguration("target_frame"),
                        "transform_timeout_sec": LaunchConfiguration("transform_timeout_sec"),
                        "fallback_to_latest_transform": LaunchConfiguration(
                            "fallback_to_latest_transform"
                        ),
                    },
                ],
            ),
        ]
    )
