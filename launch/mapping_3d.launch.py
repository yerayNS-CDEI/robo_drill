import os

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, PythonExpression
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():

    ros_domain_id = os.getenv('ROS_DOMAIN_ID')
    if int(ros_domain_id) not in range(1, 20):
        raise ValueError(
            f"ROS_DOMAIN_ID must be in the range [1, 19]. Current value: {ros_domain_id}. \n"
            "  Please set ROS_DOMAIN_ID using one of the following commands:\n"
            "  - set_moby_model GREEN\n"
            "  - set_moby_model RED\n"
            "  - export ROS_DOMAIN_ID=<value>"
        )

    package_path = FindPackageShare("robo_drill")
    rviz_config_file = PathJoinSubstitution([package_path, 'rviz/mapping_3D.rviz'])

    declared_arguments = [
        DeclareLaunchArgument("sim", default_value="false", description="simulation mode"),
        DeclareLaunchArgument("world", default_value="castelldefels_indoors_empty", description="world file for simulation"),
        DeclareLaunchArgument("headless", default_value="false", description="Run simulation without the Gazebo GUI."),
        DeclareLaunchArgument("odom_tf_from_controller", default_value="false",
                              description="Get odom->base_link tf from diff drive controller."),
        DeclareLaunchArgument("rtab_viz", default_value="false", description="Enable/Disable the rtab visualization tool"),
        DeclareLaunchArgument("database_name", default_value="rtabmap", description="name of rtabmap database file"),
        DeclareLaunchArgument("log_level", default_value="warn", description="Log Level"),
        DeclareLaunchArgument("oak",default_value="true",description="Whether to launch the DepthAI OAK-D camera driver"),
        DeclareLaunchArgument("mode",default_value="full",description="Robot model with manipulator or not"),
        DeclareLaunchArgument("controller_type", default_value="omni", description="Which controller to launch (diffdrive or omni)"),
        DeclareLaunchArgument("hybrid_sim", default_value="false", description="Enable hybrid simulation launch (base in sim + arm in URSim)"),
        DeclareLaunchArgument("self_filter", default_value="true",
                              description="Run robot_body_filter to drop self-points before rtabmap."),
        DeclareLaunchArgument("input_cloud_topic", default_value="/combined_cloud_filtered",
                              description="Cloud topic rtabmap subscribes to. Must match the filter's output_topic when self_filter is true."),
    ]

    simulation_mode = LaunchConfiguration('sim')
    hybrid_sim = LaunchConfiguration('hybrid_sim')
    world_name = LaunchConfiguration('world')
    headless = LaunchConfiguration('headless')
    odom_tf_from_controller = LaunchConfiguration("odom_tf_from_controller")
    rtab_viz = LaunchConfiguration('rtab_viz')
    database_name = LaunchConfiguration('database_name')
    pointcloud_frame_id = LaunchConfiguration('pointcloud_frame_id')
    log_level = LaunchConfiguration('log_level')

    # Include platform launch file
    platform_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'pokeye_mobile_manipulator.launch.py'])
        ]),
        launch_arguments={'sim': simulation_mode,
                          'world': world_name,
                          'headless': headless,
                          'odom_tf_from_controller': odom_tf_from_controller,
                          'oak': LaunchConfiguration('oak'),
                          'mode': LaunchConfiguration('mode'),
                          'controller_type': LaunchConfiguration('controller_type'),
                          'launch_rviz': 'false'
                          }.items(),
        condition=UnlessCondition(
            PythonExpression(["'", simulation_mode, "' == 'true' and '", hybrid_sim, "' == 'true'"])
        ),
    )

    hybrid_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'hybrid_simulation.launch.py'])
        ]),
        launch_arguments={
            'world': world_name,
            'headless': headless,
            'controller_type': LaunchConfiguration('controller_type'),
            'launch_rviz': 'false',
        }.items(),
        condition=IfCondition(
            PythonExpression(["'", simulation_mode, "' == 'true' and '", hybrid_sim, "' == 'true'"])
        ),
    )
    
    

    # Robot body self-filter — strips robot self-points from the combined cloud
    # before rtabmap consumes it. Uses MoveIt's ShapeMask under the hood; reads
    # the URDF directly and joint poses via TF, so no move_group required.
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

    # Launch 3D slam mapping
    slam_3d_mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([package_path, 'launch', 'rtabmap.launch.py'])
        ]),
        launch_arguments={'use_sim_time': simulation_mode,
                          'localization': 'false',
                          'controller_type': LaunchConfiguration('controller_type'),
                          'rtab_viz': rtab_viz,
                          'database_path': PathJoinSubstitution([package_path, 'maps', PythonExpression(["'", database_name, "' + '.db'"])]),
                          'input_cloud_topic': LaunchConfiguration('input_cloud_topic'),
                          }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file, '--ros-args', '--log-level', log_level],
        parameters=[{'use_sim_time': simulation_mode}],
    )


    nodes = [
        platform_launch,
        hybrid_simulation_launch,
        robot_body_filter,
        slam_3d_mapping_node,
        rviz_node,
    ]

    return LaunchDescription(declared_arguments + nodes)
