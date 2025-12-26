# Autonomous_Patrol_ROS2

# Autonomous Patrol Robot (ROS 2 + Nav2)

Autonomous mobile robot system with SLAM, localization, dynamic replanning, and mission-level autonomy.  

# Project Demonstration 

These simulations show real autonomous navigation behavior implemented in this project.

# Dynamic Obstacle Avoidance and Replanning
The robot autonomously replans its path when obstacles are introduced at runtime using Nav2 costmaps.

**Watch demo**:  
https://youtu.be/dpYOTnl9Om8

# Autonomous Goal Execution using Nav2
User-defined 2D goals set in RViz are executed autonomously with retries and recovery behaviors.

**Watch demo**:  
https://youtu.be/nLsWFlQWsSQ

# Overview

This project implements a robust autonomous patrol system for a mobile robot operating in unknown and dynamic indoor environments.

The robot is capable of:
- Probabilistic localization on a 2D occupancy grid map
- Collision-free path planning to target poses
- Waypoint-based patrol execution
- Autonomous recovery from navigation failures
- Dynamic replanning when obstacles appear
- Continuous operation without human intervention

All autonomy is built on top of the ROS2 Nav2 stack, with custom mission logic implemented in Python.

## System Architecture

Gazebo Simulation
|
v
LiDAR and Odometry
|
v
AMCL Localization
|
v
Nav2 Planner and Controller
|
v
Custom Patrol Manager (Python)
|
v
cmd_vel to Differential Drive Base

# Core Contribution: Patrol Manager Node

A custom ROS2 Python node implements mission-level autonomy beyond basic navigation.

Responsibilities:
- Load patrol waypoints from YAML files
- Publish initial pose for localization
- Interface with Nav2 using the NavigateToPose action
- Execute waypoint-based patrols
- Monitor goal execution status
- Detect timeouts and navigation failures
- Trigger recovery behaviors such as costmap clearing and retries
- Skip unreachable goals without stopping the mission
- Loop patrol execution continuously
- Log performance metrics for analysis

This converts navigation primitives into a fully autonomous patrol behavior.

# Dynamic Obstacle Handling

The system was explicitly tested with runtime obstacle insertion:
- Costmaps update in real time
- Invalid paths are discarded automatically
- New collision-free paths are generated
- The robot continues operation without manual reset

This demonstrates reactive autonomy rather than scripted motion.

# Performance Logging

Each patrol run logs structured data including:
- Waypoint index
- Target pose
- Success or failure
- Retry count
- Time to reach goal
- Recovery usage
- Timestamp

This enables quantitative evaluation of navigation robustness.

# Repository Structure

autonomous-patrol-ros2/
├── patrol_ws/ # ROS2 workspace
│ └── src/patrol_manager/
├── nav2_params/ # Navigation parameter files
├── maps/ # Occupancy grid maps
├── videos/ # Simulation demos
└── README.md


# Running the Simulation

```bash
source /opt/ros/humble/setup.bash
cd patrol_ws
colcon build
source install/setup.bash
ros2 run patrol_manager patrol_node

#Srividya Srinivas
M.S. Mechanical Engineering (Robotics and Controls)
Columbia University, NY
