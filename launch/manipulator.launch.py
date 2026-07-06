import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    """Bring up the robo_drill manipulator (the 3-stage gantry).

    This is the robo_drill analogue of arm_control/arm.launch.py: it owns the
    *manipulator* half of the robot. platform.launch.py brings up the mobile base
    only; this file publishes the full (base + gantry) geometry and starts the
    gantry_position_controller. pokeye_mobile_manipulator.launch.py starts both
    together (full mode).

    In full mode this launch owns the robot_description and the controller
    manager:
      * simulation -> spawn the robot into Gazebo, whose gz_ros2_control plugin
        provides /controller_manager (the base velocity controllers spawned by
        platform.launch.py connect to it).
      * real hardware -> start a ros2_control_node with the full description.
    """

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if ros_domain_id is None or int(ros_domain_id) not in range(1, 20):
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}. \n"
            "  Please set ROS_DOMAIN_ID using one of the following commands:\n"
            "  - set_moby_model GREEN\n"
            "  - set_moby_model RED\n"
            "  - export ROS_DOMAIN_ID=<value>"
        )

    package_path = FindPackageShare("robo_drill")
    params_file = PathJoinSubstitution([package_path, 'config/diffdrive_controllers.yaml'])

    # Declare arguments (kept aligned with platform.launch.py / robot.launch.py)
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="Use simulation mode",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="omni",
            description="Which base controller is in use (diff or omni). Only used to "
                        "keep the controller config consistent with the base.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from diff drive controller.",
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
            "publish_controller_odom_tf",
            default_value="false",
            description="Override controller YAMLs so the controller publishes odom and TF.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Use mock_components instead of the real base hardware interface.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "ethercat_interface",
            default_value="eno1",
            description="Network interface used by the base EtherCAT master.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz for the manipulator bringup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([package_path, 'rviz', 'view_robot.rviz']),
            description="RViz config file for the manipulator bringup.",
        )
    )

    # Initialize arguments
    simulation_mode = LaunchConfiguration("sim")
    controller_type = LaunchConfiguration("controller_type")
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    odom_from_controller = LaunchConfiguration("odom_from_controller")
    publish_controller_odom_tf = LaunchConfiguration("publish_controller_odom_tf")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    ethercat_interface = LaunchConfiguration("ethercat_interface")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    controller_should_publish_tf = PythonExpression(
        ["'true' if '", publish_controller_odom_tf, "' == 'true' else '", odom_tf_from_controller, "'"]
    )
    controller_should_publish_odom = PythonExpression(
        ["'true' if '", publish_controller_odom_tf, "' == 'true' else '", odom_from_controller, "'"]
    )

    # Controllers YAML consumed by the Gazebo gz_ros2_control plugin (simulation).
    # The base velocity controllers publishing TF/odom is toggled the same way as
    # in robot.launch.py / sim.launch.py.
    configured_sim_controllers = RewrittenYaml(
        source_file=params_file,
        param_rewrites={
            'should_publish_tf': controller_should_publish_tf,
            'should_publish_odom': controller_should_publish_odom,
        },
        convert_types=True,
    )

    # Full robot description (mobile base + gantry). This is the robo_drill
    # equivalent of the mobile_manipulator description used by the first robot.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [package_path, "robo_drill_description/description", "mobile_manipulator.urdf.xacro"]
            ),
            " ",
            "base_only:=false",
            " ",
            "simulation:=", simulation_mode,
            " ",
            "sim_ignition:=", simulation_mode,
            " ",
            "use_mock_hardware:=", use_mock_hardware,
            " ",
            "ethercat_interface:=", ethercat_interface,
            " ",
            "ros_domain_id:=", ros_domain_id,
            " ",
            "simulation_controllers:=", configured_sim_controllers,
        ]
    )
    robot_description = {
        "robot_description": ParameterValue(value=robot_description_content, value_type=str)
    }

    # Full-robot state publisher (replaces the base-only RSP from platform.launch.py
    # while in full mode).
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description, {'use_sim_time': simulation_mode}],
    )

    # Real hardware: we must start our own controller manager. In simulation the
    # gz_ros2_control plugin (embedded in the URDF) provides it instead.
    param_substitutions = {
        'enable_odom_tf': odom_tf_from_controller,
        'should_publish_tf': controller_should_publish_tf,
        'should_publish_odom': controller_should_publish_odom,
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
        condition=UnlessCondition(simulation_mode),
    )

    # Simulation: spawn the full robot into Gazebo. The Gazebo world itself is
    # started by platform.launch.py (sim.launch.py), so we only spawn the model.
    pkg_share_dir = get_package_share_directory('robo_drill')
    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=os.path.dirname(pkg_share_dir),
    )
    gazebo_spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'robo_drill',
                   '-x', '2.0',
                   '-y', '-2.0',
                   '-z', '0.22'],
        output='screen',
        condition=IfCondition(simulation_mode),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
    )

    gantry_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_position_controller", "--controller-manager", "controller_manager"],
    )

    # Start the gantry controller once the joint_state_broadcaster is up.
    delay_gantry_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[gantry_controller_spawner],
        )
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': simulation_mode}],
        condition=IfCondition(launch_rviz),
    )

    nodes = [
        set_gz_resource_path,
        robot_state_pub_node,
        control_node,
        gazebo_spawn_robot,
        joint_state_broadcaster_spawner,
        delay_gantry_controller_spawner,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
