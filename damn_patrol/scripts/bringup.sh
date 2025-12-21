#!/usr/bin/env bash
set -e

source /opt/ros/humble/setup.bash
export TURTLEBOT3_MODEL=burger

# Terminal 1: Gazebo
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py; exec bash"

# Terminal 2: Nav2
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; export TURTLEBOT3_MODEL=burger; ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=$HOME/turtlebot3_maps/my_map.yaml; exec bash"

# Terminal 3: Bridge
gnome-terminal -- bash -c "source /opt/ros/humble/setup.bash; python3 $HOME/cmd_vel_bridge.py; exec bash"
