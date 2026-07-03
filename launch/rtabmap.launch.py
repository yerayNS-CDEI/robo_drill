from launch import LaunchDescription, LaunchContext
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition 
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.actions import LogInfo

def launch_setup(context: LaunchContext, *args, **kwargs):

    # Get launch configurations
    use_sim_time_config = LaunchConfiguration('use_sim_time')
    use_sim_time = use_sim_time_config.perform(context)
    use_sim_time = str(use_sim_time).lower() == 'true'
    rtabmap_viz_disable = LaunchConfiguration('rtab_viz').perform(context) 
    localization = LaunchConfiguration('localization').perform(context)
    #converting string returned by the above line to boolean
    localization = localization == 'true' or localization == 'True'
    database_path = LaunchConfiguration('database_path').perform(context)
    controller_type = LaunchConfiguration('controller_type').perform(context)
    # In omni mode, localize a virtual footprint aligned with the turret heading.
    robot_base_frame = 'turret_footprint' if controller_type == 'omni' else 'base_footprint'


    pointcloud_frame_id = 'os_lidar'
    point_cloud_topic = LaunchConfiguration('input_cloud_topic').perform(context)

    # Common parameters
    wait_for_transform = 1
    imu_topic = "/front/sick_front/imu" if use_sim_time else "/front/imu"
    # Sim bridges rear camera topics without the node name segment; real driver adds it
    camera_rear_prefix = '/camera_rear' if use_sim_time else '/camera_rear/camera'
    deskewing = False

    # Prepare parameter
    extra_rtabmap_parameters = {}
    arguments = []
    
    if localization:
        extra_rtabmap_parameters['Mem/IncrementalMemory'] = 'false'
        extra_rtabmap_parameters['Mem/InitWMWithAllNodes'] = 'false'
        extra_rtabmap_parameters['RGBD/StartAtOrigin'] = 'true'  # allow localization anywhere in the map
        extra_rtabmap_parameters['Reg/Strategy'] = '2'       # ICP + VIS
        extra_rtabmap_parameters['Grid/PublishOccupancyGrid'] = 'true'
        extra_rtabmap_parameters['Grid/Global/MinSize'] = '50'

        extra_rtabmap_parameters['Vis/MinInliers'] = '10'
        extra_rtabmap_parameters['Vis/EstimationType'] = '0'
        extra_rtabmap_parameters['Vis/FeatureType'] = '6'    # None


    else:
        # For the merged lidar cloud, height-based ground filtering is more stable than
        # normal estimation, and ray tracing is required to carve free space in the map.
        extra_rtabmap_parameters['Grid/RayTracing'] = 'False'
        extra_rtabmap_parameters['Grid/NormalsSegmentation'] = 'False'
        extra_rtabmap_parameters['Grid/MaxGroundHeight'] = '0.2'
        extra_rtabmap_parameters['Grid/MinObstacleHeight'] = '0.2'
        extra_rtabmap_parameters['map_always_update'] = True   #Allows update on map while mapping process
                                                                # when mapping, if true, it removes obstacles when the lidar loses sight of them not reliable always
        # arguments.append('-d') # delete database on start

    # rtabmap slam arguments
    arguments.append('--uerror')
    arguments.append('--ros-args')
    arguments.append('--log-level')
    arguments.append('warn')
    point_cloud_topic_desk = point_cloud_topic

    # Nodes
    nodes = []

    # Optional deskewing node
    if deskewing:
        point_cloud_topic_desk = f"{point_cloud_topic}/deskewed"
        deskewing_node = Node(
            package='rtabmap_util', 
            executable='lidar_deskewing', 
            name="lidar_deskewing", 
            output="screen",
            parameters=[{
                "wait_for_transform": wait_for_transform,
                "fixed_frame_id": pointcloud_frame_id,
                "slerp": False,
                "use_sim_time": use_sim_time
            }],
            remappings=[("input_cloud", point_cloud_topic)],
            namespace="deskewing"
        )
        nodes.append(deskewing_node)

    # Realsense RGB-D Sync Node
    realsense_sync_node = Node(
        package='rtabmap_sync', 
        executable='rgbd_sync',
        output="screen",
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.01,
            "queue_size": 50,
            "use_sim_time": use_sim_time,
        }],
        namespace="camera1",
        remappings=[('depth/image', '/camera/camera/depth/image_rect_raw'),
                    ('rgb/image', '/camera/camera/color/image_raw'),
                    ('rgb/camera_info', '/camera/camera/color/camera_info'),
                    ('rgbd_image', '/camera1/rgbd_image')]
    )

    # Rear RealSense RGB-D Sync Node
    rs_rear_sync_node = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        output="screen",
        parameters=[{
            "approx_sync": True,
            "approx_sync_max_interval": 0.01,
            "use_sim_time": use_sim_time,
        }],
        namespace="camera2",
        remappings=[('depth/image', camera_rear_prefix + '/depth/image_rect_raw'),
                    ('rgb/image', camera_rear_prefix + '/color/image_raw'),
                    ('rgb/camera_info', camera_rear_prefix + '/color/camera_info'),
                    ('rgbd_image', '/camera2/rgbd_image')]
    )


    # Point Cloud aggeregator nodelet
    aggregator_node = Node(
        package='rtabmap_util',
        executable='point_cloud_aggregator',
        name='point_cloud_aggregator',
        output='screen',
        parameters=[{
            "frame_id": pointcloud_frame_id, # lidar_name arg can be avoided and fix the pointcloud_frame_id to sick or os_lidar
            "fixed_frame_id": "odom",
            "queue_size": 10,
            "count": 3, #* number of clouds to aggregate
            "wait_for_transform_duration": wait_for_transform,
            "use_sim_time": use_sim_time,
        }],
        remappings=[("cloud1", "/dome/points"),
                    ("cloud2", "/front/points"),
                    ("cloud3", "/rear/points"),
                    ],
        condition=IfCondition(use_sim_time_config),
    )
    
    
    # ICP Odometry Node
    icp_odom_node = Node(
        package='rtabmap_odom', 
        executable='icp_odometry', 
        name="icp_odometry", 
        output="screen",
        arguments = ['--ros-args', '--log-level', 'warn'],
        parameters=[{
            "frame_id": robot_base_frame,
            "odom_frame_id": "odom",
            "guess_frame_id": robot_base_frame,
            "publish_tf": True,
            "wait_imu_to_init": False,
            "wait_for_transform_duration": str(wait_for_transform),
            "Icp/PointToPlane": "True",     # Use point to plane ICP. default: true
            "Icp/Iterations": "10",         # Max iterations. Default: 30
            "Icp/VoxelSize": "0.2",         # Uniform sampling voxel size (0=disabled)
            "Icp/DownsamplingStep": "1",    # Downsampling step size (1=no sampling). This is done before uniform sampling
            "Icp/Epsilon": "0.001",         # Set the transformation epsilon (maximum allowable difference between two consecutive transformations) default:1
                                            # in order for an optimization to be considered as having converged to the final solution. Default:0
            "Icp/PointToPlaneK": "20",      # Number of neighbors to compute normals for point to plane if the cloud doesn't have already normals. Default:5
            "Icp/PointToPlaneRadius": "0",  # Search radius to compute normals for point to plane if the cloud doesn't have already normals. default:0.0
            "Icp/MaxTranslation": "2.0",      # Maximum ICP translation correction accepted (m). Default:0.2
            "Icp/MaxCorrespondenceDistance": "1", #Maximum distance between point correspondeces -> Lower value -> Increase keyframe creation. default: 0.1
            "Icp/PM": "True",
            "Icp/PMOutlierRatio": "0.1",
            "Icp/CorrespondenceRatio": "0.01",          #Percentage of required valid correspondeces -> Higuer values -> Reduce KeyFrame generation
            "Icp/ReciprocalCorrespondences": "False",   # To be a valid correspondence, the corresponding point in target cloud
                                                        # to point in source cloud should be both their closest closest correspondence. default:true
            "Reg/Force3DoF": "True",                                           
            "Odom/ScanKeyFrameThr": "0.8",              #Threshold for adding keyframes -> Lower value -> More keyframes
            "Odom/Strategy": "0",
            "OdomF2M/ScanSubtractRadius": "0.2",
            "OdomF2M/ScanMaxSize": "15000",
            "use_sim_time": use_sim_time 
        }],
        remappings=[("scan_cloud", point_cloud_topic), ("scan", "/dummy/scan")],
        namespace="rtabmap"
    )
    
        
    # Stereo Odometry Node
    stereo_odom_node = Node(
        package='rtabmap_odom',
        executable='stereo_odometry',
        output='screen',
        name="stereo_odometry",
        arguments = ['--ros-args', '--log-level', 'warn'],
        parameters=[{
            'approx_sync': True,
            'frame_id': robot_base_frame,
            'odom_frame_id': 'vo',
            'guess_frame_id': 'odom',
            'publish_tf': True,
            'use_sim_time': use_sim_time,
        }],
        remappings=[
            ('left/image_rect', '/camera/camera/infra1/image_rect_raw'),
            ('right/image_rect', '/camera/camera/infra2/image_rect_raw'),
            ('left/camera_info', '/camera/camera/infra1/camera_info'),
            ('right/camera_info', '/camera/camera/infra2/camera_info'),
            ('odom', '/stereo_odom')
        ]
    )

    # RTAB-Map SLAM Node
    rtabmap_slam_node = Node(
        package='rtabmap_slam', 
        executable='rtabmap', 
        name="rtabmap", 
        output="screen",
        parameters=[{
            "database_path": database_path,
            "frame_id": robot_base_frame,
            "subscribe_depth": False,
            "subscribe_rgb": False,
            "subscribe_rgbd": True,        # Enable camera for loop closure only
            "rgbd_cameras": 2,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "odom_sensor_sync": False,#* use closest available odometry to camera images
            "Rtabmap/DetectionRate": "10",          # (Hz). RTAB-Map will filter input images to satisfy this rate.
            "RGBD/NeighborLinkRefining": "False",   # When a new node is added to the graph, the transformation of its neighbor link
                                                    # to the previous node is refined using registration approach selected (Reg/Strategy). default:false
            "RGBD/ProximityBySpace": "False",        # Detection over locations (in Working Memory) near in space. default:true
            "RGBD/ProximityMaxGraphDepth": "0",     # Maximum depth from the current/last loop closure location and the local loop closure hypotheses. Set 0 to ignore. default:50
            "RGBD/ProximityPathMaxNeighbors": "1",  # Maximum neighbor nodes compared on each path for one-to-many proximity detection.
                                                    # Set to 0 to disable one-to-many proximity detection (by merging the laser scans. default:50
            "RGBD/AngularUpdate": "0.05",           # Minimum angular displacement (rad) to update the map. Rehearsal is done prior to this, so weights are still updated. default:0.1
            "RGBD/LinearUpdate": "0.05",            # Minimum linear displacement (m) to update the map. Rehearsal is done prior to this, so weights are still updated. default:0.1

            "Mem/NotLinkedNodesKept": "False",      # Keep not linked nodes in db (rehearsed nodes and deleted nodes). default:true
            "Mem/STMSize": "30",                    # Short-term memory size. default:10

            "Reg/Strategy": "2",                    #Defines method of registration (ICP or feature-based -> crucial when adding rgbd cameras (0=Vis, 1=Icp, 2=VisIcp). default:0
            "Reg/Force3DoF": "True",               # Force 3 degrees-of-freedom transform (3Dof: x,y and yaw). Parameters z, roll and pitch will be set to 0. default:false

            "Grid/CellSize": "0.05",                # Resolution of the occupancy grid. default:0.05
            # "Grid/RangeMax": "8",                  # Maximum range from sensor. 0=inf. default:5
            # "Grid/RangeMin": "1.0",                 #* Minimum range from sensor (increased to filter robot body). default:0
            # "GridGlobal/FootprintRadius": "0.85",       #* Robot footprint radius (m). default:0.3
            "Grid/FootprintLength": "1.8",
            "Grid/FootprintWidth": "1.5",
            "Grid/FootprintHeight": "2.0",

            "Grid/ClusterRadius": "0.6",            # [Grid/NormalsSegmentation=true] Cluster maximum radius. default:0.1
            "Grid/GroundIsObstacle": "False",       # [Grid/3D=true] Ground segmentation (Grid/NormalsSegmentation) is ignored, all points are obstacles. Use this only if you want
                                                    # an OctoMap with ground identified as an obstacle (e.g., with an UAV). default:false
            "Grid/MaxGroundHeight": "0.1",         # Maximum ground height (0=disabled). Should be set if "Grid/NormalsSegmentation" is false. default:0.0
            # "Grid/MinObstacleHeight": "0.15",       # Minimum obstacle height (filter ground and low obstacles). default:0.0
            "Grid/MaxObstacleHeight": "2.2",        # Maximum obstacles height (0=disabled). default:0.0
            "Grid/NormalsSegmentation": "True",     # Segment ground from obstacles using point normals, otherwise a fast passthrough is used. default:true
            # "Grid/Sensor": "2",                  # Sensor model to use (0=RayTracing, 1=VoxelBased, 2=RayTracing+VoxelBased). default:2

            "Optimizer/Strategy": "2",              # Graph optimization strategy: 0=TORO, 1=g2o, 2=GTSAM and 3=Ceres. default:2
            "Optimizer/GravitySigma": "0.3",        # Gravity sigma value (>=0, typically between 0.1 and 0.3). Optimization is done while preserving
                                                    # gravity orientation of the poses. This should be used only with visual/lidar inertial odometry approaches,
                                                    # for which we assume that all odometry poses are aligned with gravity. Set to 0 to disable gravity constraints.
                                                    # Currently supported only with g2o and GTSAM optimization strategies (see Optimizer/Strategy). default:0.3

            "Icp/VoxelSize": "0.3",                 # Uniform sampling voxel size (0=disabled). default:0.05
            "Icp/PointToPlane": "False",            # Use point to plane ICP. default: true
            "Icp/PointToPlaneK": "20",              # Number of neighbors to compute normals for point to plane if the cloud doesn't have already normals.default:5
            "Icp/PointToPlaneRadius": "0.0",        # Search radius to compute normals for point to plane if the cloud doesn't have already normals.default:0.0
            "Icp/Iterations": "10",                 # Max iterations. default:30
            "Icp/Epsilon": "0.001",                 # Set the transformation epsilon (maximum allowable difference between two consecutive transformations)
                                                    # in order for an optimization to be considered as having converged to the final solution.default:0
            "Icp/MaxTranslation": "2",              # Maximum ICP translation correction accepted (m). Default:0.2
            "Icp/MaxCorrespondenceDistance": "1",   # Max distance for point correspondences.default:0.1
            "Icp/PM": "True",
            "Icp/PMOutlierRatio": "0.7",
            "Icp/CorrespondenceRatio": "0.4",       # Ratio of matching correspondences to accept the transform.default:0.1
            "use_sim_time": use_sim_time,
        }, extra_rtabmap_parameters],
    remappings=[
        ("scan_cloud", point_cloud_topic),
        ("imu", imu_topic),
        ('/rtabmap/map', '/map'),
        ('rgbd_image0', '/camera1/rgbd_image'),
        ('rgbd_image1', '/camera2/rgbd_image'),
        # ("odom", "/controller/odometry")
    ],
    arguments=arguments,
    namespace="rtabmap"
)

    # RTAB-Map Visualization Node
    rtabmap_viz_node = Node(
        package='rtabmap_viz', 
        executable='rtabmap_viz', 
        name="rtabmap_viz", 
        output="screen",
        parameters=[{
            "frame_id": robot_base_frame,
            "odom_frame_id": "odom",
            "subscribe_odom_info": True,
            "subscribe_scan_cloud": True,
            "approx_sync": True,
            "use_sim_time": use_sim_time 
        }, extra_rtabmap_parameters],
        remappings=[("scan_cloud", point_cloud_topic_desk)],
        condition=IfCondition(rtabmap_viz_disable),
        namespace="rtabmap"
    )

    nodes.append(realsense_sync_node)
    nodes.append(aggregator_node)
    nodes.append(rs_rear_sync_node)
    nodes.append(icp_odom_node)
    # nodes.append(stereo_odom_node)  # Disabled: Gazebo RealSense plugin doesn't provide proper stereo baseline
    nodes.append(rtabmap_slam_node)
    nodes.append(rtabmap_viz_node)


    return nodes

def generate_launch_description():
    ld = LaunchDescription()
    
    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true', 
        description='Use simulation time'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'localization', 
        default_value='true', 
        description='Enable localization mode'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'rtab_viz', 
        default_value='false', 
        description='Enable visualization rtabmap'
    ))
    
    ld.add_action(DeclareLaunchArgument(
        'pointcloud_frame_id',
        default_value='os_lidar', 
        description='Frame id used as reference for point cloud'
    ))

    ld.add_action(DeclareLaunchArgument(
        'controller_type',
        default_value='diff',
        description='Which controller to launch (diffdrive or omni)'
    ))

    ld.add_action(DeclareLaunchArgument(
        'input_cloud_topic',
        default_value='/combined_cloud_filtered',
        description='Point cloud topic consumed by icp_odometry and rtabmap. '
                    'Default expects robo_drill robot_body_filter to be running upstream.'
    ))

    # Add OpaqueFunction to process launch arguments and create nodes
    ld.add_action(OpaqueFunction(function=launch_setup))
    
    return ld
