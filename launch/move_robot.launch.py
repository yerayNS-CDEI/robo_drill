import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals, IfCondition, UnlessCondition
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

    # Launch arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "map",
            default_value="warehouse_map",
            description="default map yaml file",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "rtab_viz",
            default_value="false",
            description="Run rtabmap visualization node",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "sim",
            default_value="false",
            description="whether to run in simulation mode",
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
            "localizer",
            default_value="rtab",
            description="Localizer to get map->odom transform. slam/amcl/rtab",
        )
    )


    declared_arguments.append(
        DeclareLaunchArgument(
            "odom_tf_from_controller",
            default_value="false",
            description="Get odom->base_link tf from controller",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "log_level",
            default_value="warn",
            description="Log Level",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "database_name",
            default_value="rtabmap_fsm",
            description="name of RTABMAP database file",
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
            "mode",
            default_value="full",
            description="Robot model with manipulator or not"
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "controller_type",
            default_value="omni",
            description="Which controller to launch (diffdrive or omni)",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "planner_backend",
            default_value="legacy",
            description="Planner backend to use: legacy or moveit",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "custom_recovery_behaviors",
            default_value="true",
            description="Enable the custom arm-retract + back-out recovery behaviors "
                        "(true). Only take effect when mode==full (they need the "
                        "manipulator). Set false to disable them (stock nav2 "
                        "recoveries only), e.g. while the wall-scanning FSM is in control.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_planning_pipeline",
            default_value="pilz_industrial_motion_planner",
            description="MoveIt planning pipeline for the arm stack",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_pose_planner_id",
            default_value="PTP",
            description="MoveIt planner id for pose goals",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "moveit_joint_planner_id",
            default_value="PTP",
            description="MoveIt planner id for joint goals",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_color_profile",
            default_value="640x480x15",
            description="RealSense RGB stream profile as widthxheightxfps",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "realsense_depth_profile",
            default_value="640x480x15",
            description="RealSense depth stream profile as widthxheightxfps",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "self_filter",
            default_value="true",
            description="Run robot_body_filter to drop self-points before rtabmap.",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "input_cloud_topic",
            default_value="/combined_cloud_filtered",
            description="Cloud topic rtabmap subscribes to. Must match the filter's output_topic when self_filter is true.",
        )
    )

    # get paths
    package_path = FindPackageShare("robo_drill")
    slam_params = PathJoinSubstitution([package_path, 'config/mapper_params_online_async.yaml'])
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/navigation.rviz'])

    map_name = LaunchConfiguration('map')
    simulation_mode = LaunchConfiguration('sim')
    planner_backend = LaunchConfiguration('planner_backend')
    moveit_planning_pipeline = LaunchConfiguration('moveit_planning_pipeline')
    moveit_pose_planner_id = LaunchConfiguration('moveit_pose_planner_id')
    moveit_joint_planner_id = LaunchConfiguration('moveit_joint_planner_id')
    realsense_color_profile = LaunchConfiguration('realsense_color_profile')
    realsense_depth_profile = LaunchConfiguration('realsense_depth_profile')
    rtab_viz = LaunchConfiguration('rtab_viz')
    database_name = LaunchConfiguration('database_name')
    world_file = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    localizer = LaunchConfiguration('localizer')
    slam_mode = LaunchConfiguration("slam_mode")
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    log_level = LaunchConfiguration('log_level')
    controller_type = LaunchConfiguration("controller_type")

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'mode': slam_mode,
        'map_file_name': map_name}  ## add full path to map yaml file somehow

    configured_slam_params =RewrittenYaml(
            source_file=slam_params,
            param_rewrites=param_substitutions,
            convert_types=True)

    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'pokeye_mobile_manipulator.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                        'world': world_file,
                        'headless': headless,
                        'odom_tf_from_controller': odom_tf_from_controller,
                        'oak': LaunchConfiguration('oak'),
                        'mode': LaunchConfiguration('mode'),
                        'controller_type': controller_type,
                        'planner_backend': planner_backend,
                        'moveit_planning_pipeline': moveit_planning_pipeline,
                        'moveit_pose_planner_id': moveit_pose_planner_id,
                        'moveit_joint_planner_id': moveit_joint_planner_id,
                        'realsense_color_profile': realsense_color_profile,
                        'realsense_depth_profile': realsense_depth_profile,
                        'launch_rviz': PythonExpression(
                            ["'true' if '", planner_backend, "' == 'moveit' else 'false'"]
                        ),
                        'rviz_config_file': rviz_config_file,
                        }.items(),
    )

        # Include map launch file
    map_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'map.launch.py'])
        ]),
        launch_arguments={'map': map_name,
                          'use_sim_time': simulation_mode}.items(),
        condition=LaunchConfigurationNotEquals('localizer', 'rtab')
        )

    amcl_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'amcl_launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode}.items(),
        condition=LaunchConfigurationEquals('localizer', 'amcl')
    )

    slam_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('slam_toolbox'), 'launch', 'online_async_launch.py']),
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'slam_params_file': configured_slam_params}.items(),
        condition=LaunchConfigurationEquals('localizer', 'slam')
    )

    # Robot body self-filter — strips robot self-points from the combined cloud
    # before rtabmap consumes it. Same node used during mapping; required here
    # too because rtabmap subscribes to /combined_cloud_filtered by default.
    robot_body_filter = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'robot_body_filter.launch.py'])
        ]),
        launch_arguments={
            'use_sim_time': simulation_mode,
            'mode': LaunchConfiguration('mode'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('self_filter')),
    )

    # Launch 3d mapping when mapping mode is set to 3d
    rtab_localizer_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'rtabmap.launch.py'])
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'localization': 'true',
                          'controller_type': controller_type,
                          'rtab_viz': rtab_viz,
                          'database_path': PathJoinSubstitution([package_path, 'maps', PythonExpression(["'", database_name, "' + '.db'"])]),
                          'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                          }.items(),
        condition=LaunchConfigurationEquals('localizer', 'rtab')
    )

    nav2_params = os.path.join(
    get_package_share_directory('robo_drill'),
    'config', 'nav2_params.yaml'
)
    

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'navigation_launch.py'])
        ]),
        launch_arguments={
            # 'params_file': nav2_params,
            'use_sim_time': simulation_mode,
            'autostart': 'true',
            'controller_type': controller_type,
            'planner_backend': planner_backend,
            # The custom arm-retract/back-out recoveries need the manipulator, so
            # they are only enabled when mode==full (and the user hasn't disabled
            # them via custom_recovery_behaviors).
            'custom_recovery_behaviors': PythonExpression([
                "'true' if ('", LaunchConfiguration('custom_recovery_behaviors'),
                "' == 'true' and '", LaunchConfiguration('mode'),
                "' == 'full') else 'false'"])
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': simulation_mode}],
        condition=UnlessCondition(PythonExpression(["'", planner_backend, "' == 'moveit'"])),
    )

    nodes = [
        platform_launch,
        amcl_localizer_launch,
        slam_localizer_launch,
        robot_body_filter,
        rtab_localizer_launch,
        map_launch,
        navigation_launch,

        rviz_node
    ]

    # Launch them all!
    return LaunchDescription(declared_arguments+nodes)
