# Oliwall: Autonomous non-invasive scanning and identification robotic system

An autonomous navigation robot package for ROS2 that performs 3D mapping using RTABMap and autonomous navigation using Nav2.

## Table of Contents
- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Setup](#setup)
- [Launch Files](#launch-files)
  - [3D Mapping](#3d-mapping-mapping_3dlaunchpy)
  - [Navigation](#navigation-move_robotlaunchpy)
  - [Simulation](#simulation-simlaunchpy)
  - [Platform Control](#platform-control-platformlaunchpy)
- [Workflow](#workflow)
- [Planned Features](#planned-features)
- [Troubleshooting](#troubleshooting)

## Overview

This project provides a complete autonomous navigation solution for a differential drive robot equipped with:
- 3D LiDAR sensor (Ouster Dome or SICK)
- Two depth cameras for visual feature detection and point cloud colorization
- U10 robotic arm mounted on top for wall scanning applications
- IMU and odometry sensors

### Robot Capabilities

The robot can:
1. **Map** the environment in 3D using RTABMap SLAM
2. **Navigate** autonomously within the created map using Nav2
3. **Scan walls** using the U10 arm equipped with various sensors (GPR, multispectral camera, etc.)

### Sensor Configuration

**Dual Camera Setup:**
- Two depth cameras are mounted on the robot to assist RTABMap with:
  - **Feature detection**: Rich visual features help RTABMap build robust 3D maps
  - **Loop closure**: Visual features enable the system to recognize previously visited locations and correct drift
  - **Localization**: Improved pose estimation by fusing visual and LiDAR data
  - **Point cloud colorization**: With accurate extrinsic calibration, the cameras provide RGB information to color the 3D LiDAR point clouds, creating visually rich maps

**U10 Robotic Arm:**
- Mounted on top of the robot platform
- Designed for wall scanning applications
- Compatible with multiple sensor payloads:
  - Ground Penetrating Radar (GPR)
  - Multispectral cameras
  - Other inspection sensors
- Enables automated wall inspection and data collection

## Prerequisites

- ROS2 (Humble or later)
- RTABMap ROS
- Nav2
- Gazebo (for simulation)
- Required sensors (for real robot):
  - 3D LiDAR (Ouster Dome or SICK)
  - Two OAK-D cameras
  - Joystick controller
  - U10 arm (for wall scanning applications)

## Setup

### Environment Configuration

Before launching any nodes, you **must** set the ROS_DOMAIN_ID:

```bash
# For specific robot models (if using physical robots)
set_moby_model GREEN   # Sets ROS_DOMAIN_ID for GREEN robot
# OR
set_moby_model RED     # Sets ROS_DOMAIN_ID for RED robot

# OR set manually
export ROS_DOMAIN_ID=1  # Must be in range [1-19]
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select robo_drill
source install/setup.bash
```

---

## Launch Files

### 3D Mapping (`mapping_3d.launch.py`)

Creates a 3D map of the environment using RTABMap SLAM with visual-LiDAR fusion.

#### Purpose
- Launch the robot platform (simulation or real)
- Start RTABMap in mapping mode with dual camera support
- Enable loop closure detection using visual features
- Create colorized 3D point cloud maps
- Visualize mapping process in RViz

#### Usage

**Simulation Mode:**
```bash
ros2 launch robo_drill mapping_3d.launch.py sim:=true world:=warehouse
```

**Real Robot:**
```bash
ros2 launch robo_drill mapping_3d.launch.py sim:=false lidar:=dome
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim` | `false` | Enable simulation mode |
| `world` | `warehouse` | World file for simulation (e.g., warehouse, office) |
| `lidar` | `dome` | LiDAR sensor type (`dome` or `sick`) |
| `oak` | `true` | Enable OAK-D camera drivers (both cameras) |
| `odom_tf_from_controller` | `false` | Get odom→base_link TF from diff drive controller |
| `rtab_viz` | `false` | Launch RTABMap visualization tool |
| `database_name` | `rtabmap` | Name of RTABMap database file (saved in `maps/` folder) |
| `log_level` | `warn` | ROS logging level |

#### Examples

```bash
# Map in simulation with visualization
ros2 launch robo_drill mapping_3d.launch.py sim:=true rtab_viz:=true

# Map with real robot using SICK LiDAR, custom database name
ros2 launch robo_drill mapping_3d.launch.py sim:=false lidar:=sick database_name:=warehouse_map

# Map without OAK-D cameras (LiDAR-only mapping)
ros2 launch robo_drill mapping_3d.launch.py sim:=false oak:=false
```

#### Output
- RTABMap database saved to: `maps/<database_name>.db`
- Database contains:
  - 3D point cloud map (colorized if cameras are enabled)
  - Visual features for loop closure
  - Graph-based map representation
- Use joystick to teleoperate the robot while mapping

---

### Navigation (`move_robot.launch.py`)

Navigate autonomously within a previously created map using Nav2 with visual-aided localization.

#### Purpose
- Launch the robot platform
- Load a previously created map
- Start localization with dual camera support (AMCL, SLAM Toolbox, or RTABMap)
- Start Nav2 navigation stack
- Provide RViz interface for setting navigation goals

#### Usage

**With RTABMap Localization (default - recommended):**
```bash
ros2 launch robo_drill move_robot.launch.py sim:=true database_name:=rtabmap
```

**With AMCL Localization:**
```bash
ros2 launch robo_drill move_robot.launch.py sim:=true localizer:=amcl \
map:=warehouse_map
```

**Real Robot:**
```bash
ros2 launch robo_drill move_robot.launch.py sim:=false localizer:=rtab \
database_name:=warehouse_map
```

#### Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `sim` | `false` | Enable simulation mode |
| `world` | `warehouse` | World file for simulation |
| `localizer` | `rtab` | Localization method (`rtab`, `amcl`, or `slam`) |
| `map` | `warehouse_map` | Map YAML file name (for AMCL/SLAM modes) |
| `database_name` | `rtabmap` | RTABMap database name (for RTABMap localization) |
| `lidar` | `dome` | LiDAR sensor type (`dome` or `sick`) |
| `oak` | `true` | Enable OAK-D camera drivers (both cameras) |
| `odom_tf_from_controller` | `false` | Get odom→base_link TF from controller |
| `rtab_viz` | `false` | Launch RTABMap visualization tool |
| `log_level` | `warn` | ROS logging level |

#### Localization Methods

1. **RTABMap (`rtab`)** - Visual-LiDAR localization (recommended)
   - Uses the 3D map with visual features created during mapping phase
   - Best for 3D feature-rich environments
   - Utilizes dual cameras for robust loop closure and drift correction
   - Requires database created with `mapping_3d.launch.py`

2. **AMCL (`amcl`)** - Adaptive Monte Carlo Localization
   - Uses 2D occupancy grid map
   - LiDAR-based localization only
   - Requires a `.yaml` map file in `maps/` folder

3. **SLAM Toolbox (`slam`)** - Simultaneous mapping and localization
   - Creates/updates 2D map while navigating
   - LiDAR-based only

#### Examples

```bash
# Navigate in simulation with RTABMap (visual + LiDAR localization)
ros2 launch robo_drill move_robot.launch.py sim:=true localizer:=rtab \
database_name:=my_map

# Navigate with AMCL on real robot
ros2 launch robo_drill move_robot.launch.py sim:=false localizer:=amcl map:=office_map

# Navigate with SLAM Toolbox (mapping + navigation)
ros2 launch robo_drill move_robot.launch.py sim:=true localizer:=slam

# Navigate with RTABMap visualization to see loop closures
ros2 launch robo_drill move_robot.launch.py sim:=false localizer:=rtab rtab_viz:=true
```

#### Navigation in RViz

1. Launch the navigation stack
2. In RViz, wait for the map to load
3. Click "2D Pose Estimate" and set initial pose (if using AMCL)
4. Click "Nav2 Goal" and set navigation goal on the map
5. Robot will plan path and navigate autonomously
6. Visual features help maintain accurate localization during navigation

---

### Simulation (`sim.launch.py`)

Launch Gazebo simulation environment with the robot.

#### Usage
```bash
ros2 launch robo_drill sim.launch.py world:=warehouse lidar:=dome
```

#### Parameters
- `world`: World name (default: `warehouse`)
- `lidar`: LiDAR type (`dome` or `sick`)
- `headless`: Run Gazebo without GUI (default: `false`)

---

### Platform Control (`platform.launch.py`)

Low-level platform launch that starts robot hardware/simulation and sensors.

#### Usage
```bash
# Simulation
ros2 launch robo_drill platform.launch.py sim:=true world:=warehouse

# Real robot
ros2 launch robo_drill platform.launch.py sim:=false
```

#### Parameters
- `sim`: Simulation mode (default: `false`)
- `world`: World file for simulation (default: `warehouse`)
- `sick`: Enable SICK LiDAR (default: `false`)
- `dome`: Enable Dome LiDAR (default: `true`)
- `oak`: Enable OAK-D cameras (default: `true`)
- `odom_tf_from_controller`: TF from controller (default: `false`)

---

## Workflow

### Complete Mapping and Navigation Workflow

#### Step 1: Create a Map

```bash
# Set ROS domain
export ROS_DOMAIN_ID=1

# Launch mapping (simulation) with cameras enabled
ros2 launch robo_drill mapping_3d.launch.py sim:=true \
 world:=warehouse database_name:=my_warehouse_map

# Drive the robot around using joystick to map the entire area
# The dual cameras will:
#   - Detect visual features for loop closure
#   - Colorize the 3D point cloud
#   - Improve localization accuracy
# Monitor mapping progress in RViz
# Watch for loop closures in RTABMap (if rtab_viz:=true)
# Press Ctrl+C when mapping is complete
```

The map will be saved to `maps/my_warehouse_map.db` with:
- Colorized 3D point cloud
- Visual features for localization
- Graph structure with loop closures

#### Step 2: Navigate in the Map

```bash
# Launch navigation with the created map
ros2 launch robo_drill move_robot.launch.py sim:=true localizer:=rtab \
database_name:=my_warehouse_map

# In RViz:
# 1. Wait for map to load (you'll see colorized point cloud)
# 2. Set navigation goals using "Nav2 Goal" tool
# 3. Robot will navigate autonomously
# 4. Visual features help maintain accurate pose estimation
```

### Real Robot Workflow

```bash
# Step 1: Create map on real robot with calibrated cameras
export ROS_DOMAIN_ID=1
ros2 launch robo_drill mapping_3d.launch.py sim:=false lidar:=dome \
database_name:=real_environment

# Drive around with joystick until map is complete
# Ensure good camera-LiDAR calibration for accurate colorization
# Ctrl+C to stop

# Step 2: Navigate in real environment with visual localization
ros2 launch robo_drill move_robot.launch.py sim:=false localizer:=rtab \
database_name:=real_environment
```

### Wall Scanning Workflow (with U10 Arm)

```bash
# Step 1: Navigate to target wall
ros2 launch robo_drill move_robot.launch.py sim:=false localizer:=rtab \
database_name:=building_map
# Set navigation goal near target wall in RViz

# Step 2: Position robot and execute wall scan
# (Wall scanning launch files and procedures to be documented)
# The U10 arm will scan the wall with mounted sensors (GPR, multispectral, etc.)
```

---

## Planned Features

### Frontier-Based Exploration
Autonomous map generation through frontier-based exploration is planned for future releases. This will allow the robot to:
- Automatically explore unknown areas
- Build maps without manual teleoperation
- Intelligently choose exploration targets
- Leverage visual features for better exploration decisions

---

## Important Note: Multi-Camera RTABMap Setup

> **⚠️ CRITICAL:** To support the dual-camera setup in RTABMap, you must build RTABMap from source with OpenGV support. The binary installation does not include multi-camera synchronization features.

### Building RTABMap with Multi-Camera Support

Follow these steps to build RTABMap with OpenGV support:

#### STEP 1: Install Dependencies
```bash
sudo apt-get install build-essential cmake libeigen3-dev
sudo apt remove ros-$ROS_DISTRO-rtabmap*  # Uninstall rtabmap binaries
```

#### STEP 2: Clone OpenGV
```bash
cd ~/
git clone https://github.com/laurentkneip/opengv.git
```

#### STEP 3: Build and Install OpenGV
```bash
cd opengv
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)  # If your computer has >16GB RAM. Otherwise use -j2 or -j1 (slower)
sudo make install
```

#### STEP 4: Clone RTABMap Core Library
```bash
mkdir -p ~/rtabmap_ws/src
cd ~/rtabmap_ws/src
git clone https://github.com/introlab/rtabmap.git
```

#### STEP 5: Build RTABMap with OpenGV
```bash
cd rtabmap/build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local
make -j$(nproc)  # If your computer has >16GB RAM. Otherwise use -j2 or -j1 (slower)
sudo make install
```

#### STEP 6: Build rtabmap_ros with Multi-Camera Support
```bash
cd ~/rtabmap_ws
git clone --branch ros2 https://github.com/introlab/rtabmap_ros.git src/rtabmap_ros
rosdep update && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DRTABMAP_SYNC_MULTI_RGBD=ON \
-DRTABMAP_SYNC_USER_DATA=ON -DCMAKE_BUILD_TYPE=Release
```

#### STEP 7: Source the Workspace
```bash
source ~/rtabmap_ws/install/setup.bash
# Add this to your ~/.bashrc to make it permanent
echo "source ~/rtabmap_ws/install/setup.bash" >> ~/.bashrc
```

> **Note:** The flags `-DRTABMAP_SYNC_MULTI_RGBD=ON` and `-DRTABMAP_SYNC_USER_DATA=ON` are essential for enabling multi-camera synchronization in RTABMap.

---

## Additional Resources

### Directory Structure
```
robo-drill/
├── config/           # Configuration files for controllers, Nav2, RTABMap
│   ├── nav2_params.yaml
│   ├── rtab_ekf_params.yaml
│   └── oak_params.yaml
├── launch/           # Launch files
├── maps/             # Saved maps and databases
├── rviz/             # RViz configuration files
├── worlds/           # Gazebo world files
├── robo_drill_description/  # Robot URDF/xacro files
└── python_nodes/     # Custom Python nodes
```

### Key Topics

**Sensor Topics:**
- `/scan` - 2D laser scan
- `/points` - 3D point cloud from LiDAR
- `/camera/rgb/image_raw` - RGB camera image (camera 1)
- `/camera/depth/image_raw` - Depth image (camera 1)
- `/camera2/rgb/image_raw` - RGB camera image (camera 2)
- `/camera2/depth/image_raw` - Depth image (camera 2)

**RTABMap Topics:**
- `/rtabmap/cloud_map` - Colorized 3D point cloud map
- `/rtabmap/mapData` - Graph data with loop closures
- `/rtabmap/info` - SLAM statistics and loop closure info

**Control Topics:**
- `/cmd_vel` - Velocity commands
- `/diffbot_base_controller/cmd_vel_unstamped` - Controller input
- `/cmd_vel_joy` - Joystick velocity commands

**Navigation Topics:**
- `/map` - Occupancy grid map
- `/goal_pose` - Navigation goal
- `/local_costmap/costmap` - Local planning costmap
- `/global_costmap/costmap` - Global planning costmap

**Arm Topics (U10):**
- Topics for wall scanning operations (to be documented)


## Install libmodbus for LC3 lifting column

```bash
sudo apt update
sudo apt install -y libmodbus-dev
```

# SOEM Installation Guide for ROS2 Projects

This guide shows how to install the SOEM (Simple Open EtherCAT Master) library system-wide and integrate it with ROS2 packages.

## Prerequisites

- Ubuntu Linux system with administrative privileges
- CMake 3.28 or later
- GCC compiler
- Ninja build tool
- Git

## Installation Steps

### 1. Upgrade CMake (if needed)

SOEM requires CMake 3.28 or later. Check your version:
```bash
cmake --version
```

If your version is older than 3.28, upgrade using Kitware's official repository:

```bash
# Add Kitware's GPG key
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /usr/share/keyrings/kitware-archive-keyring.gpg >/dev/null

# Add Kitware's repository
echo "deb [signed-by=/usr/share/keyrings/kitware-archive-keyring.gpg] https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/kitware.list >/dev/null

# Update and upgrade CMake
sudo apt update && sudo apt install --only-upgrade cmake -y
```

### 2. Install Build Dependencies

```bash
sudo apt install cmake build-essential ninja-build -y
```

**Note:** The SOEM documentation has a typo - it says `build-ninja` but the correct package name is `ninja-build`.

### 3. Clone SOEM Repository

Clone SOEM to your home directory (not /tmp to avoid losing it on restart):

```bash
cd ~
git clone https://github.com/OpenEtherCATsociety/SOEM.git
cd SOEM
```

### 4. Build and Install SOEM System-Wide

**Important:** Build with `-fPIC` (Position Independent Code) flag to enable linking with ROS2 shared libraries:

```bash
# Configure with -fPIC enabled
cmake --preset default \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON

# Build
cmake --build --preset default

# Install system-wide (requires sudo)
cd build/default
sudo cmake --install .
```

### 5. Verify Installation

Check that SOEM is installed:

```bash
ls -la /usr/local/lib/libsoem.a
ls -la /usr/local/include/soem/
ls -la /usr/local/cmake/soemConfig.cmake
```

## Integration with ROS2 Package

### Update CMakeLists.txt

Add the following to your ROS2 package's `CMakeLists.txt`:

**1. Find SOEM package** (after other `find_package` calls):

```cmake
# Add SOEM
list(APPEND CMAKE_PREFIX_PATH "/usr/local/cmake")
find_package(soem REQUIRED)
```

**2. Link SOEM to your target** (after `ament_target_dependencies`):

```cmake
# Link SOEM library
target_link_libraries(your_target_name PUBLIC soem)
```

### Include SOEM in Your C++ Code

In your header file:

```cpp
#include "rclcpp_lifecycle/state.hpp"
#include "your_package/visibility_control.h"

// SOEM EtherCAT library
#include <soem/soem.h>

#include "your_other_headers.hpp"
```

You can use either:
- `#include <soem/soem.h>` - Main header (includes everything)
- `#include <soem/ethercat.h>` - Specific header

SOEM headers already include `extern "C"` guards, so no need to wrap them explicitly.

### Build Your ROS2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select your_package_name
```

## Installed Files

After installation, SOEM provides:

- **Library:** `/usr/local/lib/libsoem.a`
- **Headers:** `/usr/local/include/soem/`
  - `soem.h` - Main header
  - `ethercat.h`, `ec_*.h` - Specific components
  - `osal.h`, `nicdrv.h` - OS and network drivers
- **CMake config:** `/usr/local/cmake/soemConfig.cmake`
- **Sample binaries:** `/usr/local/bin/`
  - `ec_sample` - Basic EtherCAT sample
  - `slaveinfo` - Display slave information
  - `eepromtool` - EEPROM management
  - Others: `eoe_test`, `eni_test`, `firm_update`, `simple_ng`

## Sample Code Reference

Example SOEM usage can be found in:
- Source: `~/SOEM/test/linux/` (if you cloned to home directory)
- Installed samples: `/usr/local/bin/ec_sample`

## Troubleshooting

### Build Error: "relocation R_X86_64_PC32 against symbol... can not be used when making a shared object"

This means SOEM was built without the `-fPIC` flag. Rebuild SOEM:

```bash
cd ~/SOEM
rm -rf build
cmake --preset default \
  -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DCMAKE_POSITION_INDEPENDENT_CODE=ON
cmake --build --preset default
cd build/default
sudo cmake --install .
```

### Running Sample Binaries

Sample binaries need raw socket access. Either:

**Option 1:** Grant capabilities (recommended):
```bash
sudo setcap cap_net_raw+ep /usr/local/bin/ec_sample
/usr/local/bin/ec_sample <arguments>
```

**Option 2:** Run with sudo:
```bash
sudo /usr/local/bin/ec_sample <arguments>
```

Note: You must rerun `setcap` every time you rebuild/reinstall the binary.

## Notes

- SOEM source is in `~/SOEM` (permanent location)
- System-wide installation is in `/usr/local/*`
- The CMake config file enables `find_package(soem)` in your projects
- SOEM is a C library but works seamlessly with C++ projects

---
