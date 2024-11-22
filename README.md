# Waiter Bot Workspace

## Overview
This repository contains the ROS 2 workspace for the **Waiter Bot**, a robotic system designed to assist in navigation and mapping tasks. It includes the following features:
- SLAM for autonomous mapping.
- Navigation with Nav2.
- Robot state publishing and visualization using RViz2.

## Workspace Structure
```bash
ros_ws/ 
├── src/ │ 
├── waiter_bot_description/ │ 
│ ├── config/ # Configuration files for navigation and costmaps │ 
│ ├── launch/ # Launch files (Python) for starting nodes │ 
│ ├── navigation/ # Navigation-specific configurations │ 
│ ├── urdf/ # Robot description files (URDF/Xacro) │ 
│ ├── rviz_config/ # RViz configuration files │ 
│ └── package.xml # Package manifest └── build/, install/, log/ # ROS 2 workspace build directories
```

## Setup Instructions

### Prerequisites
Ensure the following dependencies are installed:
- ROS 2 Humble (or a compatible ROS 2 distribution)
- Navigation2 stack (`sudo apt install ros-<distro>-navigation2`)
- Slam Toolbox (`sudo apt install ros-<distro>-slam-toolbox`)
- RViz2 (`sudo apt install ros-<distro>-rviz2`)

### Build the Workspace
```bash
# Clone the repository
git clone https://github.com/yourusername/ros_ws.git
cd ros_ws

# Build the workspace
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```
### Package Details
- waiter_bot_description: 
This package includes the robot's description, configurations, and navigation setup.

#### Key Directories
- urdf/: Contains robot description files in URDF and Xacro format.
- config/: Stores navigation and planner parameter files.
- launch/: Includes Python-based launch files for running the robot and navigation stack.
- rviz_config/: RViz2 visualization configuration files.

#### Dependencies
- Build: ament_cmake, rclcpp, xacro
- Runtime: urdf, rviz2, nav2_costmap_2d, slam_toolbox
  
#### Launch Files
Here is an overview of the Python-based launch files and their purposes:

| Launch File         | Description                                       |
|---------------------|---------------------------------------------------|
| auto_navigation.py  | Automatically starts mapping and navigation.      |
| gmapping.py         | Sets up SLAM using Slam Toolbox.                  |
| bot_amcl.py         | Starts the AMCL-based localization node.          |
| move_base_mapless.py| Starts navigation without a static map.           |
| movebase.py         | Main navigation launch for the Move Base node.    |
| navigation.py       | Combines all nodes for mapping, localization, and navigation. |

Example Command:
```bash
ros2 launch waiter_bot_description navigation.py
```
### Key Nodes and Parameters
#### Nodes

##### `slam_toolbox`
- **Purpose**: Used for SLAM.
- **Parameters**:
  - `base_frame`: Base link frame.
  - `odom_frame`: Odometry frame.
- **Configured in**: `gmapping.py`

##### `nav2_amcl`
- **Purpose**: AMCL-based localization.
- **Parameters**:
  - `min_particles`: Minimum particle count.
  - `max_particles`: Maximum particle count.
- **Configured in**: `bot_amcl.py`

##### `nav2_bringup`
- **Purpose**: Handles navigation and planning tasks.
- **Configured in**: `movebase.py`

### Parameters

Key configuration files include:
- `config/planner.yaml`: Planner settings.
- `config/costmap_global.yaml`: Global costmap parameters.
- `config/costmap_local.yaml`: Local costmap parameters.


Usage Instructions
Build the workspace: 
```bash 
colcon build
```
Source the workspace: 
```bash
source install/setup.bash
```
Launch the navigation stack:
```bash
ros2 launch waiter_bot_description navigation.py
```

### Contributing
Feel free to submit issues or pull requests for improvements.
