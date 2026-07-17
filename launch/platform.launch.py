import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition


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

    # Launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value='false',
            description="Use simulation mode",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "world",
            default_value="castelldefels_indoors_empty",
            description="world file for simulation",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "headless",
            default_value="false",
            description="Run simulation without the Gazebo GUI.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sick",
            default_value="true",
            description="Whether to launch sick lidar",
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
            "oak",
            default_value="true",
            description="Whether to launch the DepthAI OAK-D camera driver"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_color_profile",
            default_value="640x480x15",
            description="RealSense RGB stream profile as widthxheightxfps"
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_depth_profile",
            default_value="640x480x15",
            description="RealSense depth stream profile as widthxheightxfps"
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
    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="false",
            description="Launch RViz for the base platform bringup.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "rviz_config_file",
            default_value=PathJoinSubstitution([package_path, 'rviz', 'view_robot.rviz']),
            description="RViz config file for the base platform bringup.",
        )
    )

    simulation_mode = LaunchConfiguration('sim')
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    launch_sick = LaunchConfiguration("sick")
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    mode = LaunchConfiguration("mode")
    controller_type = LaunchConfiguration("controller_type")
    publish_controller_odom_tf = LaunchConfiguration("publish_controller_odom_tf")
    realsense_color_profile = LaunchConfiguration("realsense_color_profile")
    realsense_depth_profile = LaunchConfiguration("realsense_depth_profile")
    launch_rviz = LaunchConfiguration("launch_rviz")
    rviz_config_file = LaunchConfiguration("rviz_config_file")

    # If controller type is omni, then use the general_params_omni.yaml config file, otherwise use general_params.yaml
    general_params = PathJoinSubstitution([
        package_path, 
        'config',
        PythonExpression(["'general_params_omni.yaml' if '", controller_type, "' == 'omni' else 'general_params.yaml'"])
    ])
        
    # Include platform launch file
    sim_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'sim.launch.py'])
        ]),
        launch_arguments={
            'sim': simulation_mode,
            'world': world_name,
            'headless': headless,
            'mode': mode,
            'controller_type': controller_type,
            'publish_controller_odom_tf': publish_controller_odom_tf,
            }.items(),
        condition=IfCondition(simulation_mode)
    )

    real_platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'robot.launch.py'])
        ]),
        launch_arguments={
            'odom_tf_from_controller':odom_tf_from_controller,
            'mode': mode,
            'controller_type': controller_type,
            'publish_controller_odom_tf': publish_controller_odom_tf,
            }.items(),
        condition=UnlessCondition(simulation_mode)
    )

    sensors_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'sensors.launch.py'])
        ]),
        launch_arguments={
            'sick': launch_sick,
            'oak': LaunchConfiguration('oak'),
            'realsense_color_profile': realsense_color_profile,
            'realsense_depth_profile': realsense_depth_profile,
        }.items(),
        condition=UnlessCondition(simulation_mode)
    )
    turret_joy = Node(
        package="robo_drill",
        executable="turret_joy.py",
        name='turret_joy',
        parameters=[general_params,
                    {'use_sim_time': simulation_mode}]
    )
    turret_footprint_broadcaster = Node(
        package="robo_drill",
        executable="turret_footprint_broadcaster.py",
        name="turret_footprint_broadcaster",
        parameters=[{
            'use_sim_time': simulation_mode,
            'base_frame': 'base_footprint',
            'tracked_frame': 'turret_link',
            'published_frame': 'turret_footprint',
            'publish_rate_hz': 30.0,
            'tf_timeout_s': 0.05,
        }],
        condition=IfCondition(PythonExpression(["'", controller_type, "' == 'omni'"]))
    )
    # Launch joystick driver and teleop node
    joy_driver = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        parameters=[{
            'dev': "/dev/input/js0",
            'deadzone': 0.3,
            'autorepeat_rate': 20.0,
        }]
    )
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy_node',
        parameters=[general_params,
                    {'use_sim_time': simulation_mode}],
        remappings={('/cmd_vel', 'cmd_vel_joy')},
    )

    # Twist mulitplexer: sets priotities on command velocities coming from different sources
    # useful to have control over the robot with both joystick and the nav2 stack
    twist_mux = Node(
        package="twist_mux",
        executable="twist_mux",
        parameters=[general_params,
                    {'use_sim_time': simulation_mode}],
        remappings=[('/cmd_vel_out','/diffbot_base_controller/cmd_vel_unstamped')]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': simulation_mode}],
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'base' and '", launch_rviz, "' == 'true'"])
        ),
    )

    nodes = [
        sim_platform_launch,
        real_platform_launch,
        sensors_launch,
        turret_footprint_broadcaster,
        turret_joy,
        joy_driver,
        teleop_twist_joy_node,
        twist_mux,
        rviz_node,
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments+nodes)
