cmake_minimum_required(VERSION 3.14)
project(robo_drill)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
if(POLICY CMP0148)
  cmake_policy(SET CMP0148 OLD)
endif()

# find dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)



find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_geometry REQUIRED)
find_package(message_filters REQUIRED) 
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)

find_package(pcl_ros REQUIRED)
find_package(PCL REQUIRED)
find_package(tf2 REQUIRED)

find_package(nav2_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behaviors REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(action_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(MODBUS REQUIRED libmodbus)

# Self-filter dependencies (ShapeMask in moveit_ros_perception). We build the
# moveit RobotModel directly from URDF+empty-SRDF, so RobotModelLoader (which
# lives in moveit_ros_planning) is not needed.
find_package(moveit_core REQUIRED)
# moveit_ros_perception exports a mesh_filter target that depends on GLUT::GLUT;
# find_package(GLUT) must precede it so the imported target resolves.
find_package(GLUT REQUIRED)
find_package(moveit_ros_perception REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(urdfdom REQUIRED)
find_package(srdfdom REQUIRED)
# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${MODBUS_INCLUDE_DIRS}
)

add_definitions(${PCL_DEFINITIONS})

add_definitions(${PCL_DEFINITIONS})

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(${EIGEN3_INCLUDE_DIRS})

set(msg_files
  "msg/Frontier.msg"
  "msg/Frontiers.msg"
  "msg/Wall.msg"
  "msg/WallArray.msg"
)

set(srv_files
  "srv/SetPosition.srv"
)
set(action_files
  "action/RetractArmFromObstacle.action"
  "action/BackOutFromObstacle.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs nav_msgs builtin_interfaces action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

add_executable(exploration_closest_frontier src/exploration_closest_frontier.cpp)
add_executable(find_frontiers src/find_frontiers.cpp)

ament_target_dependencies(find_frontiers
  rclcpp sensor_msgs nav2_msgs visualization_msgs std_msgs tf2_ros tf2_geometry_msgs
)
ament_target_dependencies(exploration_closest_frontier
  rclcpp sensor_msgs nav2_msgs visualization_msgs geometry_msgs tf2_ros tf2_geometry_msgs
)

# Build pointcloud_concatenate library
add_library(pointcloud_concatenate SHARED
  src/pointcloud_concatenate.cpp
)
ament_target_dependencies(pointcloud_concatenate
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  Eigen3
)

# Link with the generated message interfaces from this package
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(find_frontiers "${cpp_typesupport_target}")
target_link_libraries(exploration_closest_frontier "${cpp_typesupport_target}")

# Add include directory for generated headers
target_include_directories(find_frontiers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
target_include_directories(exploration_closest_frontier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Ensure message headers are built before executables that use them
add_dependencies(find_frontiers ${PROJECT_NAME}__rosidl_typesupport_cpp)
add_dependencies(exploration_closest_frontier ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Link generated message library (fixes undefined reference)
target_link_libraries(find_frontiers ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_link_libraries(exploration_closest_frontier ${PROJECT_NAME}__rosidl_typesupport_cpp)

target_link_libraries(pointcloud_concatenate
  ${PCL_LIBRARIES}
)



foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

# Build frontier_search library
add_library(frontier_search SHARED
  src/frontier_search.cpp
  src/costmap_client.cpp
)

ament_target_dependencies(frontier_search
  rclcpp
  geometry_msgs
  nav2_costmap_2d
  nav2_util
  visualization_msgs
)

add_executable(explore src/explore.cpp)
ament_target_dependencies(explore
  rclcpp
  nav2_msgs
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  visualization_msgs
  rclcpp_action
  std_msgs
  tf2_ros
  tf2_geometry_msgs
)


add_executable(robot_body_filter_node
  src/robot_body_filter_node.cpp
)
ament_target_dependencies(robot_body_filter_node
  rclcpp
  sensor_msgs
  tf2
  tf2_ros
  tf2_eigen
  moveit_core
  moveit_ros_perception
  geometric_shapes
  urdfdom
  srdfdom
  Eigen3
)

add_executable(pointcloud_concatenate_node
  src/pointcloud_concatenate_node.cpp
)
ament_target_dependencies(pointcloud_concatenate_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  Eigen3
)

target_link_libraries(pointcloud_concatenate_node
  pointcloud_concatenate
  ${PCL_LIBRARIES}
)

# Tier-A wall detection node (gravity-aligned 2.5D line extraction)
add_executable(wall_detection_node
  src/wall_detection_node.cpp
)
ament_target_dependencies(wall_detection_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  pcl_conversions
  pcl_ros
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
  OpenCV
)
target_link_libraries(wall_detection_node
  ${PCL_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
add_dependencies(wall_detection_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(wall_detection_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Tier-C persistent wall aggregator (semantic layer on top of RTAB-Map's pose)
add_executable(wall_aggregator_node
  src/wall_aggregator_node.cpp
)
ament_target_dependencies(wall_aggregator_node
  rclcpp
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_eigen
  Eigen3
  OpenCV
  ament_index_cpp
)
target_link_libraries(wall_aggregator_node
  ${OpenCV_LIBRARIES}
  ${PROJECT_NAME}__rosidl_typesupport_cpp
)
add_dependencies(wall_aggregator_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
target_include_directories(wall_aggregator_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)

# Column control executable

add_executable(column_control
  src/column_control.cpp
)
ament_target_dependencies(column_control
  rclcpp
  std_msgs
  std_srvs
)
target_link_libraries(column_control
  ${MODBUS_LIBRARIES}
)
target_include_directories(column_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(column_control ${PROJECT_NAME}__rosidl_typesupport_cpp)
# Link with the generated service interface
rosidl_get_typesupport_target(cpp_typesupport_target_column
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(column_control "${cpp_typesupport_target_column}")


target_link_libraries(explore frontier_search)
add_library(
  robodrill_hw_interface
  SHARED
  hardware/src/robodrill_hardware_interface.cpp
  hardware/src/column_hardware_interface.cpp
)

target_compile_features(robodrill_hw_interface PUBLIC cxx_std_17)
target_include_directories(robodrill_hw_interface PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/hardware/include>
  $<INSTALL_INTERFACE:include/robodrill_hw_interface>
  ${SOEM_INCLUDE_DIR}
)

ament_target_dependencies(
   robodrill_hw_interface PUBLIC
   ${THIS_PACKAGE_INCLUDE_DEPENDS}
 )

target_compile_definitions(robodrill_hw_interface PRIVATE "ROBODRILL_HW_INTERFACE_BUILDING_DLL")

# Find SOEM library
find_library(SOEM_LIBRARY 
  NAMES soem libsoem 
  PATHS /usr/local/lib 
  NO_DEFAULT_PATH
)
find_path(SOEM_INCLUDE_DIR 
  soem/soem.h 
  PATHS /usr/local/include 
  NO_DEFAULT_PATH
)


# Find libmodbus
find_library(MODBUS_LIBRARY 
  NAMES modbus libmodbus
)
find_path(MODBUS_INCLUDE_DIR
  modbus/modbus.h
)


# Add modbus include directory
target_include_directories(robodrill_hw_interface PUBLIC
  ${MODBUS_INCLUDE_DIR}
)

target_link_libraries(robodrill_hw_interface 
  PUBLIC 
  ${SOEM_LIBRARY}
  ${MODBUS_LIBRARY}
)


# Create symbolic link for Gz Sim meshes
set(INPUT_MESHES_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(OUTPUT_MESHES_SYMLINK $ENV{HOME}/.gz/models/${PROJECT_NAME})
execute_process(COMMAND ${CMAKE_COMMAND} -E create_symlink ${INPUT_MESHES_DIR} ${OUTPUT_MESHES_SYMLINK})

# Export the nav2_core behavior plugin description (RetractArmFromObstacle + BackOutFromObstacle).
pluginlib_export_plugin_description_file(nav2_core robo_drill_behavior_plugin.xml)

# ---- "Back out from obstacle" recovery behavior ----
# Direction-aware base-strafe escape. Nav2 behavior plugin (behavior_server).
add_library(robo_drill_back_out_behavior SHARED
  src/back_out_from_obstacle.cpp
)
ament_target_dependencies(robo_drill_back_out_behavior
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_util
  pluginlib
)
target_link_libraries(robo_drill_back_out_behavior "${cpp_typesupport_target}")
target_include_directories(robo_drill_back_out_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(robo_drill_back_out_behavior ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Behavior-tree action node (loaded by bt_navigator via plugin_lib_names).
add_library(robo_drill_back_out_bt_node SHARED
  src/back_out_from_obstacle_bt_node.cpp
)
ament_target_dependencies(robo_drill_back_out_bt_node
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp_v3
)
target_compile_definitions(robo_drill_back_out_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(robo_drill_back_out_bt_node "${cpp_typesupport_target}")
target_include_directories(robo_drill_back_out_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(robo_drill_back_out_bt_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# ---- "Retract arm from obstacle" recovery behavior ----
# Nav2 behavior plugin (loaded by behavior_server).
add_library(robo_drill_retract_arm_behavior SHARED
  src/retract_arm_from_obstacle.cpp
)
ament_target_dependencies(robo_drill_retract_arm_behavior
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_geometry_msgs
  nav2_core
  nav2_behaviors
  nav2_costmap_2d
  nav2_util
  pluginlib
)
target_link_libraries(robo_drill_retract_arm_behavior "${cpp_typesupport_target}")
target_include_directories(robo_drill_retract_arm_behavior PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(robo_drill_retract_arm_behavior ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Behavior-tree action node (loaded by bt_navigator via plugin_lib_names).
add_library(robo_drill_retract_arm_bt_node SHARED
  src/retract_arm_from_obstacle_bt_node.cpp
)
ament_target_dependencies(robo_drill_retract_arm_bt_node
  rclcpp
  rclcpp_action
  nav2_behavior_tree
  behaviortree_cpp_v3
)
target_compile_definitions(robo_drill_retract_arm_bt_node PRIVATE BT_PLUGIN_EXPORT)
target_link_libraries(robo_drill_retract_arm_bt_node "${cpp_typesupport_target}")
target_include_directories(robo_drill_retract_arm_bt_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp>
)
add_dependencies(robo_drill_retract_arm_bt_node ${PROJECT_NAME}__rosidl_typesupport_cpp)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface hardware/robodrill_hw_interface_plugin.xml)
pluginlib_export_plugin_description_file(hardware_interface hardware/column_hw_interface_plugin.xml)


# Install executables
install(TARGETS
  explore
  exploration_closest_frontier
  find_frontiers
  pointcloud_concatenate_node
  robot_body_filter_node
  wall_detection_node
  wall_aggregator_node
  column_control
  DESTINATION lib/${PROJECT_NAME}
)

# Install libraries
install(TARGETS
  frontier_search
  pointcloud_concatenate
  robo_drill_retract_arm_behavior
  robo_drill_retract_arm_bt_node
  robo_drill_back_out_behavior
  robo_drill_back_out_bt_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install header files
install(DIRECTORY include/
  DESTINATION include
)

# Keep the detections directory present in the source tree so symlink installs
# don't fail if it was removed while empty.
file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/rgb_detections")

# INSTALL
install(
  DIRECTORY robo_drill_description launch rviz worlds maps rgb_detections config
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY hardware/include/
  DESTINATION include/robodrill_hw_interface
)

install(TARGETS robodrill_hw_interface
  EXPORT export_robodrill_hw_interface
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

#Install python modules
# ament_python_install_package(${PROJECT_NAME})  # Commented out - conflicts with rosidl message generation

# Install python executables
install(PROGRAMS
    python_nodes/turret_joy.py
    python_nodes/turret_footprint_broadcaster.py
    python_nodes/start_pose_publisher.py
    python_nodes/goal_pose_publisher.py
    python_nodes/cancel_goals.py
    python_nodes/follow_path.py
    python_nodes/follow_poses.py
    python_nodes/republish_lidar.py
    python_nodes/goal_router.py
    python_nodes/yolo_object_detection.py
    python_nodes/odom_plot.py
    python_nodes/dynamic_footprint_publisher.py
    utils/imu_to_yaw.py

    DESTINATION lib/${PROJECT_NAME})

# Symlink the Robot Control UI as `UI` (no extension) so it runs as
# `ros2 run robo_drill UI` and stays live-editable with --symlink-install.
install(CODE "execute_process(COMMAND \"${CMAKE_COMMAND}\" -E create_symlink \"${CMAKE_CURRENT_SOURCE_DIR}/ui/UI.py\" \"${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/UI\")")

# Install the UI's helper package (UI_utils) next to the UI executable so
# `from UI_utils.qtermwidget_wrapper import QTermWidget` resolves at runtime.
install(DIRECTORY ui/UI_utils
    DESTINATION lib/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.py")

ament_export_dependencies(rosidl_default_runtime)
ament_package()