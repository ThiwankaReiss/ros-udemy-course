# ROS 2 Learning Workspace

This repository contains my **tutorial/practice work** while learning ROS 2.

## Important Note

This is **not fully original self work** and should be treated as **tutorial-level learning material**. Most of the code, structure, and ideas were followed from ROS learning resources and adapted for practice.

## Purpose

- Practice ROS 2 package structure and development flow
- Learn C++ and Python ROS 2 node patterns
- Explore actions, lifecycle nodes, executors, composition, launch files, and SLAM workflows

## Repository Overview

### Core learning packages

- `actions_cpp` - ROS 2 action server/client examples in C++
- `actions_py` - ROS 2 action server/client examples in Python
- `components_cpp` - component composition examples in C++
- `components_py` - component composition examples in Python
- `executors_cpp` - single-threaded and multi-threaded executors in C++
- `executors_py` - single-threaded and multi-threaded executors in Python
- `lifecycle_cpp` - lifecycle node examples in C++
- `lifecycle_py` - lifecycle node examples in Python

### Robot-specific and integration packages

- `my_robot_interfaces` - custom interfaces (`msg`, `srv`, `action`)
- `my_robot_description` - robot URDF/description files and visualization assets
- `my_robot_bringup` - launch files, RViz configs, and Gazebo world setup

### SLAM experiments

- `slam` - scripts and experiments related to navigation/SLAM

## Build

From the ROS 2 workspace root (the directory above `src`):

```bash
colcon build
```

Then source the workspace:

```bash
# Linux/macOS
source install/setup.bash

# Windows PowerShell
.\install\setup.ps1
```

## Run examples

Use ROS 2 launch and run commands after sourcing:

```bash
ros2 launch my_robot_bringup components.launch.py
```

```bash
ros2 run actions_py count_until_server
```

```bash
ros2 run actions_py count_until_client
```

## Who this repo is for

- Beginners who want reference material while learning ROS 2 concepts
- Anyone who wants small, focused examples in both C++ and Python

## Attribution

This repository is educational and based on tutorial-style learning content. Please do not treat it as a production-grade or fully original implementation.
