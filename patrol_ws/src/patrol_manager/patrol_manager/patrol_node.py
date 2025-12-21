#!/usr/bin/env python3
import os
import csv
import math
import time
import threading
from dataclasses import dataclass
from typing import List, Tuple, Optional

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap


def yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class Waypoint:
    x: float
    y: float
    yaw: float


class DamnPatrolNode(Node):
    """
    Stable architecture:
      - Main thread runs rclpy.spin(node) (executor lives here)
      - Patrol runs in a background thread but NEVER calls rclpy.spin* functions
      - Background thread waits by polling futures + sleeping
    """

    def __init__(self):
        super().__init__("damn_patrol_manager")

        # ---------- Params ----------
        self.declare_parameter("waypoints_file", os.path.expanduser("~/damn_patrol/config/waypoints.yaml"))
        self.declare_parameter("loop_forever", True)
        self.declare_parameter("max_retries_per_goal", 2)
        self.declare_parameter("goal_timeout_sec", 180.0)

        self.declare_parameter("autopublish_initialpose", True)
        self.declare_parameter("initialpose_x", -2.0)
        self.declare_parameter("initialpose_y", -0.5)
        self.declare_parameter("initialpose_yaw", 0.0)
        self.declare_parameter("initialpose_wait_sec", 8.0)

        self.declare_parameter("metrics_csv", os.path.expanduser("~/damn_patrol/results/patrol_metrics.csv"))

        self.waypoints_file = os.path.expanduser(self.get_parameter("waypoints_file").value)
        self.loop_forever = bool(self.get_parameter("loop_forever").value)
        self.max_retries = int(self.get_parameter("max_retries_per_goal").value)
        self.goal_timeout_sec = float(self.get_parameter("goal_timeout_sec").value)
        self.metrics_csv = os.path.expanduser(self.get_parameter("metrics_csv").value)

        os.makedirs(os.path.dirname(self.metrics_csv), exist_ok=True)

        # ---------- Action + services ----------
        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")

        self.clear_global = self.create_client(ClearEntireCostmap, "/global_costmap/clear_entirely_global_costmap")
        self.clear_local = self.create_client(ClearEntireCostmap, "/local_costmap/clear_entirely_local_costmap")

        # initialpose publisher QoS: TRANSIENT_LOCAL
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, "/initialpose", qos)

        # ---------- Load YAML waypoints ----------
        self.frame_id, self.waypoints = self._load_waypoints(self.waypoints_file)
        if not self.waypoints:
            raise RuntimeError(f"No waypoints loaded from {self.waypoints_file}")

        self.get_logger().info(f"loop_forever={self.loop_forever}, max_retries_per_goal={self.max_retries}")

        # ---------- Metrics ----------
        self._init_metrics()

        # bringup state
        self._started = False
        self._nav_ready_time = None

        # start timer
        self.create_timer(0.5, self._tick)

    # -------- Utility: wait on future WITHOUT spinning (thread-safe) --------
    def _wait_future(self, fut, timeout_sec: float) -> bool:
        t_end = time.time() + timeout_sec
        while time.time() < t_end and rclpy.ok():
            if fut.done():
                return True
            time.sleep(0.02)
        return fut.done()

    # -------- YAML --------
    def _load_waypoints(self, path: str) -> Tuple[str, List[Waypoint]]:
        if not os.path.exists(path):
            self.get_logger().error(f"Waypoints YAML not found: {path}")
            return ("map", [])

        with open(path, "r") as f:
            data = yaml.safe_load(f)

        frame_id = data.get("frame_id", "map")
        if "loop_forever" in data:
            self.loop_forever = bool(data["loop_forever"])
        if "max_retries_per_goal" in data:
            self.max_retries = int(data["max_retries_per_goal"])

        wps = []
        for item in data.get("waypoints", []):
            wps.append(Waypoint(float(item["x"]), float(item["y"]), float(item.get("yaw", 0.0))))

        self.get_logger().info(f"Loaded {len(wps)} waypoints from {path} (frame_id={frame_id})")
        return frame_id, wps

    # -------- Metrics --------
    def _init_metrics(self):
        if not os.path.exists(self.metrics_csv):
            with open(self.metrics_csv, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow([
                    "attempt_idx", "waypoint_idx",
                    "x", "y", "yaw",
                    "success", "retries_used",
                    "time_to_goal_sec", "recovery_used",
                    "note", "stamp_sec"
                ])

    def _log_metric(self, attempt_idx, wp_idx, wp: Waypoint, success, retries_used, dt, recovery_used, note):
        with open(self.metrics_csv, "a", newline="") as f:
            w = csv.writer(f)
            w.writerow([
                attempt_idx, wp_idx,
                f"{wp.x:.3f}", f"{wp.y:.3f}", f"{wp.yaw:.3f}",
                str(success), retries_used,
                f"{dt:.3f}", str(recovery_used),
                note, f"{time.time():.3f}"
            ])

    # -------- Bringup tick (non-blocking) --------
    def _tick(self):
        if self._started:
            return

        if self._nav_ready_time is None:
            if self.nav_client.wait_for_server(timeout_sec=0.5):
                self._nav_ready_time = time.time()
                self.get_logger().info("NavigateToPose action server detected. Warming up graph...")
            else:
                self.get_logger().info("Waiting for /navigate_to_pose action server...")
                return

        if time.time() - self._nav_ready_time < 3.0:
            return

        if not self.clear_global.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Waiting for global costmap clear service...")
            return
        if not self.clear_local.wait_for_service(timeout_sec=0.2):
            self.get_logger().warn("Waiting for local costmap clear service...")
            return

        if bool(self.get_parameter("autopublish_initialpose").value):
            self._publish_initialpose()
            wait_s = float(self.get_parameter("initialpose_wait_sec").value)
            self.get_logger().info(f"Waiting {wait_s:.1f}s for AMCL to settle...")
            # small sleep loop (still not spinning from a different thread)
            t_end = time.time() + wait_s
            while time.time() < t_end and rclpy.ok():
                time.sleep(0.05)

        self._started = True
        self.get_logger().info("=== DAMN PATROL STARTED ===")

        threading.Thread(target=self._run_patrol, daemon=True).start()

    def _publish_initialpose(self):
        x = float(self.get_parameter("initialpose_x").value)
        y = float(self.get_parameter("initialpose_y").value)
        yaw = float(self.get_parameter("initialpose_yaw").value)
        qx, qy, qz, qw = yaw_to_quat(yaw)

        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = self.frame_id
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw

        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.0685

        self.initialpose_pub.publish(msg)
        self.get_logger().info(f"Published /initialpose x={x:.2f}, y={y:.2f}, yaw={yaw:.2f} rad (frame={self.frame_id})")

    # -------- Recovery --------
    def _clear_costmaps(self) -> bool:
        req = ClearEntireCostmap.Request()
        ok = True
        for client, name in [(self.clear_global, "global"), (self.clear_local, "local")]:
            fut = client.call_async(req)
            done = self._wait_future(fut, timeout_sec=5.0)
            if done and fut.result() is not None:
                self.get_logger().warn(f"[RECOVERY] Cleared {name} costmap")
            else:
                self.get_logger().error(f"[RECOVERY] Failed to clear {name} costmap")
                ok = False
        return ok

    # -------- Navigation --------
    def _send_goal_and_wait(self, wp: Waypoint):
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = self.frame_id
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = wp.x
        goal.pose.pose.position.y = wp.y
        qx, qy, qz, qw = yaw_to_quat(wp.yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw

        t0 = time.time()

        send_fut = self.nav_client.send_goal_async(goal)
        if not self._wait_future(send_fut, timeout_sec=15.0):
            return False, "send_goal_timeout", time.time() - t0

        if send_fut.exception() is not None:
            return False, f"send_goal_exception:{type(send_fut.exception()).__name__}", time.time() - t0

        goal_handle = send_fut.result()
        if goal_handle is None:
            return False, "send_goal_none", time.time() - t0
        if not goal_handle.accepted:
            return False, "goal_rejected", time.time() - t0

        result_fut = goal_handle.get_result_async()
        if not self._wait_future(result_fut, timeout_sec=self.goal_timeout_sec):
            # cancel but do not spin; just fire request and continue
            goal_handle.cancel_goal_async()
            return False, "timeout_waiting_result", time.time() - t0

        res = result_fut.result()
        if res is None:
            return False, "no_result", time.time() - t0

        status = res.status
        dt = time.time() - t0

        if status == GoalStatus.STATUS_SUCCEEDED:
            return True, "succeeded", dt
        return False, f"status_{status}", dt

    def _run_patrol(self):
        attempt_idx = 0
        lap = 0

        while rclpy.ok():
            self.get_logger().info(f"--- Patrol lap {lap} ---")

            for i, wp in enumerate(self.waypoints):
                retries_used = 0
                recovery_used = False

                self.get_logger().info(f"[NAV] Waypoint {i}: x={wp.x:.2f}, y={wp.y:.2f}, yaw={wp.yaw:.2f}")
                success, note, dt = self._send_goal_and_wait(wp)

                while (not success) and (retries_used < self.max_retries) and rclpy.ok():
                    retries_used += 1
                    recovery_used = True
                    self.get_logger().warn(f"[NAV] Failed ({note}). Recovery+retry {retries_used}/{self.max_retries} ...")
                    self._clear_costmaps()
                    time.sleep(0.5)
                    success, note, dt = self._send_goal_and_wait(wp)

                self._log_metric(attempt_idx, i, wp, success, retries_used, dt, recovery_used, note)

                if success:
                    self.get_logger().info(f"[OK] Reached waypoint {i} in {dt:.1f}s (retries={retries_used})")
                else:
                    self.get_logger().error(f"[SKIP] Waypoint {i} failed after retries. note={note}")

                attempt_idx += 1
                time.sleep(0.2)

            lap += 1
            if not self.loop_forever:
                break


def main():
    rclpy.init()
    node = DamnPatrolNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
