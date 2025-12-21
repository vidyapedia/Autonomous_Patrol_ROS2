# Autonomous_Patroll_ROS2

# Autonomous Patrol Robot (ROS 2 + Nav2)

Autonomous waypoint patrol in a mapped environment using ROS 2 Humble + Nav2.
Tested in Gazebo + RViz using TurtleBot3.

## Demo Videos
- Manual localization + Nav2 goals: (add link)
- Autonomous patrol + dynamic obstacles: (add link)

## What it does
- Loads a saved map
- Localizes using AMCL
- Navigates through waypoint list repeatedly
- Replans around obstacles
- Logs metrics (time-to-goal, retries, failures)

## Tech
ROS 2 Humble • Nav2 • Gazebo • RViz2 • Python (rclpy)

## How to run (high-level)
1) Launch Gazebo TurtleBot3 world  
2) Launch Nav2 with map  
3) Run patrol manager node with waypoints YAML  
4) Watch in RViz + record video

## Repo structure (to be filled after upload)
- `patrol_ws/src/patrol_manager/` (your custom node)
- `damn_patrol/config/waypoints.yaml`
- `nav2_params/burger_fixed.yaml`
- `results/` (metrics csv)
