from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription

def generate_launch_description():
    # Declare the launch argument for the bag file path
    bag_file_path = DeclareLaunchArgument(
        'bag_file_path',
        default_value='',
        description='Path to the ROS2 bag file'
    )
    use_sim_time = DeclareLaunchArgument(
        'sim',
        default_value='true',
        description='use simulation time'
    )

    # robot state publisher because we cant directly get thre tree from bag. tf_static topic should contain the whole tree but since
    # turret is not a static tf our tree remains broken if read from the bag.
    # there launching robot state publisher and having joint states published from the bag
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([get_package_share_directory('robo_drill'), 'launch', 'robot_state_publisher.launch.py'])
        ]),
        launch_arguments={'sim': LaunchConfiguration('sim')}.items(),
    )

    # Define the process to play the bag file with the specified arguments
    play_bag_process = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play', LaunchConfiguration('bag_file_path'),
            '--topics', '/joint_states', '/imu', '/scan', '/points', '/gps/fix', '/gps/navheading', '/controller/odometry', '--clock'
        ],
        output='screen'
    )

    return LaunchDescription([
        bag_file_path,
        use_sim_time,
        robot_state_publisher,
        play_bag_process,

    ])
