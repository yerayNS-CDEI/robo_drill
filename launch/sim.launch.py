import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.substitutions import FindExecutable, Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import UnlessCondition, IfCondition
from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    # Check if ros_domain_id is not in range 1-19
    if int(ros_domain_id) not in range(1, 20):  # Check if it is a string within range [1, 19]
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}. \n"
            "  Please set ROS_DOMAIN_ID using one of the following commands:\n"
            "  - set_moby_model GREEN\n"
            "  - set_moby_model RED\n"
            "  - export ROS_DOMAIN_ID=<value>"
        )

    package_path = FindPackageShare("robo_drill")

    # Set IGN_GAZEBO_RESOURCE_PATH so Gz Sim can resolve package:// mesh URIs
    # package:// gets converted to model:// by Gz Sim, which looks up IGN_GAZEBO_RESOURCE_PATH
    pkg_share_dir = get_package_share_directory('robo_drill')
    gz_resource_path = os.path.dirname(pkg_share_dir)  # .../install/robo_drill/share/
    set_gz_resource_path = AppendEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=gz_resource_path,
    )

    # Declaring arguments of the launch file
    declared_arguments = []
    declared_arguments.append(DeclareLaunchArgument('sim', default_value='true', description='Simulation mode (should always be true for this file)'))
    declared_arguments.append(DeclareLaunchArgument('world', default_value='warehouse', description='world file name'))
    declared_arguments.append(DeclareLaunchArgument('headless', default_value='false', description='whether to launch gazebo client'))
    declared_arguments.append(DeclareLaunchArgument('mode', default_value='full', description='Launch mode full|base', choices=['full', 'base']))
    declared_arguments.append(DeclareLaunchArgument('controller_type', default_value='diff', description='Which controller to launch (diffdrive or omni)'))
    declared_arguments.append(
        DeclareLaunchArgument(
            'publish_controller_odom_tf',
            default_value='false',
            description='Override controller YAMLs so the controller publishes odom and TF.',
        )
    )

    simulation_mode = LaunchConfiguration('sim')
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    mode = LaunchConfiguration('mode')
    controller_type = LaunchConfiguration('controller_type')
    publish_controller_odom_tf = LaunchConfiguration('publish_controller_odom_tf')

    world_dir = PathJoinSubstitution([package_path, 'worlds', world_name])
    world_file_name = PathJoinSubstitution([world_dir, PythonExpression(["'", world_name, "' + '.sdf'"])])
    configured_sim_controllers = RewrittenYaml(
        source_file=PathJoinSubstitution([package_path, 'config', 'diffdrive_controllers.yaml']),
        param_rewrites={
            'should_publish_tf': publish_controller_odom_tf,
            'should_publish_odom': publish_controller_odom_tf,
        },
        convert_types=True,
    )

    # The robot state publisher node with the robot description
    robot_description_content = Command(
        # The idea is to run the whole thing as a command you would type in the terminal
        [FindExecutable(name="xacro"), # get path to the executable /opt/ros/humble/bin/xacro
        " ", # add space
        PathJoinSubstitution([package_path, "robo_drill_description/description", "robo_drill.urdf.xacro"]), # get path to the robot description xacro file
        " ",
        "simulation:=true",
        " ",
        "ros_domain_id:=", ros_domain_id,
        " ",
        "use_mock_hardware:=true",
        " ",
        "base_only:=", PythonExpression(["'true' if '", mode, "' == 'base' else 'false'"]),
        " ",
        "simulation_controllers:=", configured_sim_controllers,
         ]
    )
    robot_description = {'robot_description': robot_description_content}
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'base'"])
        )
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
        condition=UnlessCondition(headless),
    )

    gz_sim_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py'
            ])
        ),
        launch_arguments={
            'gz_args': ['-r -s ', world_file_name],
        }.items(),
        condition=IfCondition(headless),
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    # Spawn controllers based on controller_type
    sim_controller_spawner_diff = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diffbot_base_controller", "turret_controller", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'diff'"]))
    )

    sim_controller_spawner_omni = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["sim_controller", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'omni'"]))
    )

    column_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["column_controller", "--controller-manager", "controller_manager"],
    )

    gantry_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["gantry_position_controller", "--controller-manager", "controller_manager"],
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    # Run the spawner node from the ros_gz_sim package.
    spawn_robo_drill = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                    '-name', 'robo_drill',
                    '-x', '2.0',
                    '-y', '-2.0',
                    '-z', '0.22'],
        output='screen',
        condition=IfCondition(PythonExpression(["'", mode, "' == 'base'"]))
    )

    # Bridge /clock and sensor topics from Gz to ROS (simulation only)
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args', '-p',
            f'config_file:={os.path.join(pkg_share_dir, "config", "gz_bridge.yaml")}',
        ],
        output='screen',
        condition=IfCondition(simulation_mode),
    )

    # Use ros_gz_image for efficient camera image bridging (simulation only - bridges from Gazebo)
    camera_color_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/camera/color/image_raw'],
        output='screen',
        condition=IfCondition(simulation_mode),
    )
    camera_depth_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/camera/depth/image_rect_raw'],
        output='screen',
        condition=IfCondition(simulation_mode),
    )

    # Rear camera (back of turret) - color + depth images bridged via ros_gz_image
    camera_rear_color_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_rear/color/image_raw'],
        output='screen',
        condition=IfCondition(simulation_mode),
    )
    camera_rear_depth_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera_rear/depth/image_rect_raw'],
        output='screen',
        condition=IfCondition(simulation_mode),
    )

    nodes = [
        set_gz_resource_path,
        gz_sim,
        gz_sim_headless,
        gz_bridge,
        camera_color_image_bridge,
        camera_depth_image_bridge,
        camera_rear_color_image_bridge,
        camera_rear_depth_image_bridge,
        spawn_robo_drill,
        joint_state_broadcaster_spawner,
        robot_state_publisher,
        sim_controller_spawner_diff,
        sim_controller_spawner_omni,
        column_controller_spawner,
        gantry_controller_spawner,
    ]
    
    # Launch them all!
    return LaunchDescription(declared_arguments + nodes)
