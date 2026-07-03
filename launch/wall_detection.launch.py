from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("robo_drill")
    default_params = PathJoinSubstitution([pkg, "config", "wall_detection_params.yaml"])

    params_file = LaunchConfiguration("params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    aggregate = LaunchConfiguration("aggregate")
    detector_mode = LaunchConfiguration("detector_mode")
    use_grid = LaunchConfiguration("use_grid")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=default_params),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument(
            "detector_mode", default_value="projection_hough",
            description="Wall detector front-end: projection_hough or rht_3d."),
        DeclareLaunchArgument(
            "aggregate", default_value="true",
            description="Also run the Tier-C persistent wall aggregator (needs map<-odom from SLAM)."),
        DeclareLaunchArgument(
            "use_grid", default_value="true",
            description="Aggregator output source: true = grid-anchored (RTAB-Map "
                        "grid layout, lidar confirms height); false = lidar-only "
                        "(the detector's own walls are published, best for "
                        "evaluating/tuning the detector itself)."),
        Node(
            package="robo_drill",
            executable="wall_detection_node",
            name="wall_detection_node",
            output="screen",
            parameters=[params_file, {
                "use_sim_time": use_sim_time,
                "detector_mode": detector_mode,
            }],
        ),
        Node(
            package="robo_drill",
            executable="wall_aggregator_node",
            name="wall_aggregator_node",
            output="screen",
            condition=IfCondition(aggregate),
            parameters=[params_file, {
                "use_sim_time": use_sim_time,
                "use_grid": use_grid,
            }],
        ),
    ])
