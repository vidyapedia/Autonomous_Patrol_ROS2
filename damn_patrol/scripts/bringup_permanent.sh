#!/usr/bin/env bash
# Prevent ROS setup from crashing under 'set -u'
export AMENT_TRACE_SETUP_FILES=${AMENT_TRACE_SETUP_FILES:-""}

set -eo pipefail

# --- CONFIG ---
MAP_FILE="${HOME}/turtlebot3_maps/my_map.yaml"
PARAMS_FILE="${HOME}/nav2_params/burger_fixed.yaml"

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export TURTLEBOT3_MODEL="${TURTLEBOT3_MODEL:-burger}"

# --- Helpers ---
die() { echo -e "\n[FAIL] $1\n" >&2; exit 1; }
ok()  { echo -e "[OK] $1"; }

require_file() {
  [[ -f "$1" ]] || die "Missing file: $1"
}

wait_for_topic() {
  local topic="$1"
  local tries="${2:-40}" # ~40 sec
  for _ in $(seq 1 "$tries"); do
    if ros2 topic list 2>/dev/null | grep -qx "${topic}"; then
      ok "Topic exists: ${topic}"
      return 0
    fi
    sleep 1
  done
  die "Topic did not appear: ${topic}"
}

wait_for_node_grep() {
  local pattern="$1"
  local tries="${2:-40}"
  for _ in $(seq 1 "$tries"); do
    if ros2 node list 2>/dev/null | grep -qi "${pattern}"; then
      ok "Node match found: ${pattern}"
      return 0
    fi
    sleep 1
  done
  die "Node did not appear (pattern): ${pattern}"
}

ensure_cmd_vel_single_publisher() {
  # We allow velocity_smoother as publisher. behavior_server must NOT publish /cmd_vel.
  local info
  info="$(ros2 topic info /cmd_vel -v 2>/dev/null || true)"
  echo "$info" | grep -q "Subscription count: 1" || die "/cmd_vel not subscribed by diff_drive"
  echo "$info" | grep -q "turtlebot3_diff_drive" || die "turtlebot3_diff_drive not subscribing /cmd_vel"
  echo "$info" | grep -q "Node name: velocity_smoother" || die "velocity_smoother is not publishing /cmd_vel (pipeline not correct)"
  if echo "$info" | grep -q "Node name: behavior_server"; then
    die "behavior_server is publishing /cmd_vel (cmd_vel conflict). Your params file is not being applied."
  fi
  ok "cmd_vel pipeline correct (velocity_smoother -> /cmd_vel -> diff_drive)"
}

# --- Start ---
source /opt/ros/humble/setup.bash

require_file "$MAP_FILE"
require_file "$PARAMS_FILE"

echo -e "\n=== STARTING PERMANENT BRINGUP ==="
echo "[INFO] ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
echo "[INFO] TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
echo "[INFO] MAP=$MAP_FILE"
echo "[INFO] PARAMS=$PARAMS_FILE"

# Kill any leftover ros2 processes from previous crashes
pkill -f "ros2 launch turtlebot3_gazebo" 2>/dev/null || true
pkill -f "ros2 launch turtlebot3_navigation2" 2>/dev/null || true
sleep 2

# Start Gazebo in background
nohup bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL; ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py" \
  > "${HOME}/damn_patrol_gazebo.log" 2>&1 &

ok "Launched Gazebo (log: ~/damn_patrol_gazebo.log)"

# Gate: /clock + turtlebot3 nodes must exist
wait_for_topic "/clock" 60
wait_for_node_grep "turtlebot3_diff_drive" 60
wait_for_topic "/cmd_vel" 60

# Start Nav2 in background
nohup bash -lc "source /opt/ros/humble/setup.bash; export ROS_DOMAIN_ID=$ROS_DOMAIN_ID; export TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL; ros2 launch turtlebot3_navigation2 navigation2.launch.py map:=${MAP_FILE} params_file:=${PARAMS_FILE}" \
  > "${HOME}/damn_patrol_nav2.log" 2>&1 &

ok "Launched Nav2 (log: ~/damn_patrol_nav2.log)"

# Gate: Nav2 nodes must exist
wait_for_node_grep "velocity_smoother" 60
wait_for_topic "/cmd_vel_nav" 60

# Final: ensure cmd_vel pipeline correct
ensure_cmd_vel_single_publisher

echo -e "\n=== ALL SYSTEMS UP ==="
echo "Next (RViz):"
echo "  1) Set Fixed Frame = map"
echo "  2) Click 2D Pose Estimate, set pose, wait 10s"
echo "  3) Click Nav2 Goal"
echo -e "\nLogs:"
echo "  Gazebo:  ~/damn_patrol_gazebo.log"
echo "  Nav2:    ~/damn_patrol_nav2.log"
