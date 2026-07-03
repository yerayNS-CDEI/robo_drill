import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


# Top-level xacro file is mode-dependent. Resolved inside an OpaqueFunction so
# we can read the `mode` LaunchConfiguration value and pick the matching file
# at launch time — matches how the rest of the stack composes URDFs.
def _build_node(context, *_args, **_kwargs):
    package_share = FindPackageShare("robo_drill")
    description_dir = "robo_drill_description/description"

    mode = LaunchConfiguration("mode").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context)
    lidar = LaunchConfiguration("lidar").perform(context)

    if mode == "base":
        xacro_file = "robo_drill.urdf.xacro"
        base_only = "true"
    else:
        xacro_file = "mobile_manipulator.urdf.xacro"
        base_only = "false"

    ros_domain_id = os.getenv("ROS_DOMAIN_ID", "")

    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([package_share, description_dir, xacro_file]),
            " ",
            "use_mock_hardware:=false",
            " ",
            "simulation:=", use_sim_time,
            " ",
            "ros_domain_id:=", ros_domain_id,
            " ",
            "lidar:=", lidar,
            " ",
            "base_only:=", base_only,
        ]
    )

    config_file = PathJoinSubstitution([package_share, "config", "robot_body_filter.yaml"])

    filter_node = Node(
        package="robo_drill",
        executable="robot_body_filter_node",
        name="robot_body_filter",
        output="screen",
        parameters=[
            config_file,
            {
                "robot_description": ParameterValue(robot_description_content, value_type=str),
                "use_sim_time": (use_sim_time.lower() == "true"),
            },
        ],
    )
    return [filter_node]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument(
                "mode",
                default_value="full",
                description="Robot model: 'base' uses robo_drill.urdf.xacro; "
                            "anything else uses mobile_manipulator.urdf.xacro.",
            ),
            DeclareLaunchArgument("lidar", default_value="sick"),
            OpaqueFunction(function=_build_node),
        ]
    )
