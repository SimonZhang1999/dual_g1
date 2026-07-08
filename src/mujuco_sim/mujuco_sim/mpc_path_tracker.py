#!/usr/bin/env python3
"""Sequential waypoint tracker for the leader robot.

Uses the centralized /formation/sync_wp_idx published by
formation_sync_controller so the leader and follower advance waypoints
together as a synchronized formation.

Subscribes:
  /formation/leader_reference  nav_msgs/Path       - MPC waypoint sequence
  /formation/sync_wp_idx       std_msgs/Int32      - shared waypoint index
  /formation/goal_reached      std_msgs/Bool       - stop signal
  /odom                        nav_msgs/Odometry   - robot pose

Publishes:
  /cmd_vel   geometry_msgs/Twist
"""
from __future__ import annotations

import math
from typing import List

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool, Int32


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class MPCPathTracker(Node):
    def __init__(self) -> None:
        super().__init__("mpc_path_tracker")

        self.declare_parameter("leader_reference_topic", "/formation/leader_reference")
        self.declare_parameter("sync_wp_idx_topic",      "/formation/sync_wp_idx")
        self.declare_parameter("goal_reached_topic",     "/formation/goal_reached")
        self.declare_parameter("odom_topic",             "/odom")
        self.declare_parameter("cmd_vel_topic",          "/cmd_vel")

        self.declare_parameter("max_vx_forward",  0.30)
        self.declare_parameter("max_vx_backward", 0.15)
        self.declare_parameter("max_vy",          0.20)
        self.declare_parameter("max_vyaw",        0.50)
        self.declare_parameter("kp_xy",           1.2)
        self.declare_parameter("kp_yaw",          1.5)
        self.declare_parameter("decel_radius",    0.80)

        p = self.get_parameter
        self._max_vx_fwd = float(p("max_vx_forward").value)
        self._max_vx_bwd = float(p("max_vx_backward").value)
        self._max_vy     = float(p("max_vy").value)
        self._max_w      = float(p("max_vyaw").value)
        self._kp_xy      = float(p("kp_xy").value)
        self._kp_yaw     = float(p("kp_yaw").value)
        self._decel_r    = float(p("decel_radius").value)

        # state
        self._path: List     = []
        self._sync_idx       = 0
        self._robot_x        = 0.0
        self._robot_y        = 0.0
        self._robot_yaw      = 0.0
        self._goal_done      = False

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self._cmd_pub = self.create_publisher(
            Twist, str(p("cmd_vel_topic").value), reliable)

        self.create_subscription(
            Path,    str(p("leader_reference_topic").value), self._on_path,         latched)
        self.create_subscription(
            Int32,   str(p("sync_wp_idx_topic").value),      self._on_sync_idx,     latched)
        self.create_subscription(
            Bool,    str(p("goal_reached_topic").value),     self._on_goal_reached, latched)
        self.create_subscription(
            Odometry, str(p("odom_topic").value),            self._on_odom,         reliable)

        self._timer = self.create_timer(0.05, self._control_step)  # 20 Hz
        self.get_logger().info(
            "MPC leader path tracker ready (sync mode via /formation/sync_wp_idx)"
        )

    # ── callbacks ───────────────────────────────────────────────────────
    def _on_path(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._path = list(msg.poses)
        self._goal_done = False

    def _on_sync_idx(self, msg: Int32) -> None:
        self._sync_idx = msg.data

    def _on_goal_reached(self, msg: Bool) -> None:
        if msg.data:
            self._goal_done = True
            self._publish_zero()
            self.get_logger().info("Goal reached → leader stopped")

    def _on_odom(self, msg: Odometry) -> None:
        self._robot_x   = msg.pose.pose.position.x
        self._robot_y   = msg.pose.pose.position.y
        self._robot_yaw = _yaw_from_quat(msg.pose.pose.orientation)

    # ── control loop ────────────────────────────────────────────────────
    def _control_step(self) -> None:
        if self._goal_done or not self._path:
            self._publish_zero()
            return

        idx    = min(self._sync_idx, len(self._path) - 1)
        target = self._path[idx].pose

        # Position error in robot body frame
        tx = target.position.x - self._robot_x
        ty = target.position.y - self._robot_y
        c  = math.cos(-self._robot_yaw)
        s  = math.sin(-self._robot_yaw)
        ex = c * tx - s * ty   # forward
        ey = s * tx + c * ty   # lateral

        # Yaw error toward waypoint's orientation
        wp_yaw  = _yaw_from_quat(target.orientation)
        yaw_err = _angle_diff(wp_yaw, self._robot_yaw)

        # Decelerate only at the last waypoint (terminal goal)
        is_last = (idx >= len(self._path) - 1)
        if is_last:
            dist  = math.hypot(tx, ty)
            scale = min(1.0, dist / max(0.01, self._decel_r))
        else:
            scale = 1.0

        vx = max(-self._max_vx_bwd, min(self._max_vx_fwd,
                  self._kp_xy * ex * scale))
        vy = max(-self._max_vy,     min(self._max_vy,
                  self._kp_xy * ey * scale))
        w  = max(-self._max_w,      min(self._max_w,
                  self._kp_yaw * yaw_err))

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.angular.z = w
        self._cmd_pub.publish(cmd)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MPCPathTracker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
