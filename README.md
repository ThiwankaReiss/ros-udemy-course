# Autonomous Car — ROS 2 Development Workspace

Building toward a fully autonomous vehicle using ROS 2.

## Vision

The goal is to develop a self-driving car capable of real-time perception, path planning, and safe navigation — powered by ROS 2, simulation-tested in Gazebo, and eventually deployed on physical hardware.

## What's Here

This workspace lays the foundation: mastering ROS 2 core concepts (actions, lifecycle nodes, executors, composition, SLAM) that will directly feed into the autonomous car stack.

| Area | Purpose |
|---|---|
| `actions_*` | Behaviour execution pipeline (C++ & Python) |
| `lifecycle_*` | Safe state management for vehicle nodes |
| `executors_*` | Real-time concurrency patterns |
| `components_*` | Modular, composable node architecture |
| `my_robot_*` | Robot description, interfaces, bringup & simulation |
| `slam/` | Navigation and mapping experiments |

## Build

```bash
colcon build
source install/setup.bash
```
