# ROS2 and Nav2-based Auto Patrol Robot
## 1. Project Overview
This project involves the design and simulation of an automatic patrol robot using ROS 2 and Navigation 2. The patrol robot is capable of moving cyclically between different target points. Upon reaching each target, it performs the following tasks:
1. Plays the information about the target point through voice playback.
2. Captures a real-time image using a camera and saves it locally.

**Package Breakdown:**
- fishbot_description: Robot description file, including simulation-related configurations.
- fishbot_navigation2: Robot navigation configuration files.
- fishbot_Application: Robot navigation application for motor control.
- autopatrol_interfaces: Interfaces for automatic patrol-related operations.
- autopatrol_robot: Main package for implementing the automatic patrol functionality.

## 2. Installation
This project is developed on the following platform:

- System Version: Ubuntu 22.04

- ROS Version: ROS 2 Humble
### 2.1 Install Dependencies
This project uses slam-toolbox for SLAM, Navigation 2 for navigation, Gazebo for simulation, and ros2-control for motion control. Please install the dependencies before building the project. Use the following commands:

1. Install SLAM and Navigation 2:

```shell
sudo apt install ros-$ROS_DISTRO-nav2-bringup ros-$ROS_DISTRO-slam-toolbox
```

2. Install Simulation-related Packages:
```shell
sudo apt install ros-$ROS_DISTRO-robot-state-publisher ros-$ROS_DISTRO-joint-state-publisher ros-$ROS_DISTRO-gazebo-ros-pkgs ros-$ROS_DISTRO-ros2-controllers ros-$ROS_DISTRO-xacro
```
3. Install Speech Synthesis and Image-related Packages:
```shell
sudo apt install python3-pip -y
sudo apt install espeak-ng -y
sudo pip3 install espeakng
sudo apt install ros-$ROS_DISTRO-tf-transformations
sudo pip3 install transforms3d
```
### 2.2 Build and Run
Once the dependencies are installed, you can use the colcon tool to build and run the project.
```shell
colcon build
```
Run Simulation:
```shell
source install/setup.bash
ros2 launch fishbot_description gazebo.launch.py
```
Run Navigation:
```shell
source install/setup.bash
ros2 launch fishbot_navigation2 navigation2.launch.py
```
Run Autonomous Patrol:
```shell
source install/setup.bash
ros2 launch autopatrol_robot autopatrol.launch.py
```
## 3. Author
- [Ray](https://github.com/yepraywong)