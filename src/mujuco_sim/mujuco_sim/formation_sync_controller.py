#!/usr/bin/env python3
"""Formation synchronization controller.

Coordinates waypoint advancement so the leader and follower always target
the same waypoint index.  Both robots must satisfy position AND yaw
tolerance at paths[sync_idx] before the index is incremented.

Publishes (all latched):
  /formation/sync_wp_idx   std_msgs/Int32    - shared waypoint index
  /formation/goal_reached  std_msgs/Bool     - True when formation at terminal
  /formation/sync_status   std_msgs/String   - human-readable debug info

Subscribes:
  /formation/leader_reference   nav_msgs/Path        - leader MPC waypoints
  /formation/follower_reference nav_msgs/Path        - follower MPC waypoints
  /formation/leader_goal        geometry_msgs/PoseStamped - terminal leader goal
  /odom                         nav_msgs/Odometry    - leader pose
  /follower/odom                nav_msgs/Odometry    - follower pose
"""
from __future__ import annotations

import math
from typing import List, Optional

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
)
from std_msgs.msg import Bool, Int32, String


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_diff(a: float, b: float) -> float:
    d = a - b
    return math.atan2(math.sin(d), math.cos(d))


class FormationSyncController(Node):
    def __init__(self) -> None:
        super().__init__("formation_sync_controller")

        # tolerances
        self.declare_parameter("wp_tolerance_xy",    0.35)
        self.declare_parameter("wp_tolerance_yaw",   0.40)
        self.declare_parameter("goal_tolerance_xy",  0.20)
        self.declare_parameter("goal_tolerance_yaw", 0.25)

        # topics
        self.declare_parameter("leader_odom_topic",      "/odom")
        self.declare_parameter("follower_odom_topic",    "/follower/odom")
        self.declare_parameter("leader_ref_topic",       "/formation/leader_reference")
        self.declare_parameter("follower_ref_topic",     "/formation/follower_reference")
        self.declare_parameter("leader_goal_topic",      "/formation/leader_goal")
        self.declare_parameter("sync_idx_topic",         "/formation/sync_wp_idx")
        self.declare_parameter("goal_reached_topic",     "/formation/goal_reached")
        self.declare_parameter("status_topic",           "/formation/sync_status")

        p = self.get_parameter
        self._wp_tol_xy   = float(p("wp_tolerance_xy").value)
        self._wp_tol_yaw  = float(p("wp_tolerance_yaw").value)
        self._goal_tol_xy  = float(p("goal_tolerance_xy").value)
        self._goal_tol_yaw = float(p("goal_tolerance_yaw").value)

        # state
        self._leader_path:   List[PoseStamped] = []
        self._follower_path: List[PoseStamped] = []
        self._leader_goal:   Optional[Pose] = None
        self._sync_idx  = 0
        self._active    = False
        self._goal_done = False

        self._lx = 0.0;  self._ly = 0.0;  self._lyaw = 0.0
        self._fx = 0.0;  self._fy = 0.0;  self._fyaw = 0.0

        # QoS
        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE)
        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # publishers
        self._sync_pub    = self.create_publisher(
            Int32,  str(p("sync_idx_topic").value),     latched)
        self._reached_pub = self.create_publisher(
            Bool,   str(p("goal_reached_topic").value), latched)
        self._status_pub  = self.create_publisher(
            String, str(p("status_topic").value),       reliable)

        # subscriptions
        self.create_subscription(
            Path,        str(p("leader_ref_topic").value),    self._on_leader_ref,   latched)
        self.create_subscription(
            Path,        str(p("follower_ref_topic").value),  self._on_follower_ref, latched)
        self.create_subscription(
            PoseStamped, str(p("leader_goal_topic").value),   self._on_leader_goal,  latched)
        self.create_subscription(
            Odometry,    str(p("leader_odom_topic").value),   self._on_leader_odom,  reliable)
        self.create_subscription(
            Odometry,    str(p("follower_odom_topic").value), self._on_follower_odom, reliable)

        self._timer = self.create_timer(0.1, self._sync_step)  # 10 Hz

        self.get_logger().info(
            f"Formation sync controller ready  "
            f"wp_tol=({self._wp_tol_xy:.2f}m, "
            f"{math.degrees(self._wp_tol_yaw):.1f}deg)"
        )

    # ── callbacks ───────────────────────────────────────────────────────
    def _on_leader_goal(self, msg: PoseStamped) -> None:
        self._leader_goal = msg.pose
        self._sync_idx    = 0
        self._active      = True
        self._goal_done   = False
        self._publish_idx()
        yaw = math.degrees(_yaw_from_quat(msg.pose.orientation))
        self.get_logger().info(
            f"New formation goal: pos=({msg.pose.position.x:.2f}, "
            f"{msg.pose.position.y:.2f})  yaw={yaw:.1f}deg"
        )

    def _on_leader_ref(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._leader_path = list(msg.poses)
        # Re-sync index to first unvisited waypoint after re-plan
        self._sync_idx = self._first_unvisited(
            self._leader_path, self._lx, self._ly, self._lyaw)
        self._publish_idx()

    def _on_follower_ref(self, msg: Path) -> None:
        if not msg.poses:
            return
        self._follower_path = list(msg.poses)

    def _on_leader_odom(self, msg: Odometry) -> None:
        self._lx   = msg.pose.pose.position.x
        self._ly   = msg.pose.pose.position.y
        self._lyaw = _yaw_from_quat(msg.pose.pose.orientation)

    def _on_follower_odom(self, msg: Odometry) -> None:
        self._fx   = msg.pose.pose.position.x
        self._fy   = msg.pose.pose.position.y
        self._fyaw = _yaw_from_quat(msg.pose.pose.orientation)

    # ── helpers ─────────────────────────────────────────────────────────
    def _first_unvisited(self, path: List[PoseStamped],
                          rx: float, ry: float, ryaw: float) -> int:
        """First waypoint not yet reached (pos OR yaw still unsatisfied)."""
        for i, ps in enumerate(path):
            dist = math.hypot(ps.pose.position.x - rx, ps.pose.position.y - ry)
            dyaw = abs(_angle_diff(_yaw_from_quat(ps.pose.orientation), ryaw))
            if dist >= self._wp_tol_xy or dyaw >= self._wp_tol_yaw:
                return i
        return len(path) - 1

    def _at_wp(self, rx: float, ry: float, ryaw: float,
               path: List[PoseStamped], idx: int) -> bool:
        ps   = path[idx].pose
        dist = math.hypot(ps.position.x - rx, ps.position.y - ry)
        dyaw = abs(_angle_diff(_yaw_from_quat(ps.orientation), ryaw))
        return dist < self._wp_tol_xy and dyaw < self._wp_tol_yaw

    def _publish_idx(self) -> None:
        msg = Int32()
        msg.data = self._sync_idx
        self._sync_pub.publish(msg)

    # ── sync loop ────────────────────────────────────────────────────────
    def _sync_step(self) -> None:
        if not self._active or self._goal_done:
            return
        if not self._leader_path or not self._follower_path:
            return

        n   = min(len(self._leader_path), len(self._follower_path))
        idx = min(self._sync_idx, n - 1)

        l_ok = self._at_wp(self._lx, self._ly, self._lyaw,
                            self._leader_path, idx)
        f_ok = self._at_wp(self._fx, self._fy, self._fyaw,
                            self._follower_path, idx)

        if l_ok and f_ok:
            if idx < n - 1:
                # Both reached intermediate waypoint → advance
                self._sync_idx += 1
                self.get_logger().info(
                    f"Both at wp[{idx}] → advancing to wp[{self._sync_idx}]"
                )
                self._publish_idx()
            else:
                # Both at last waypoint → check terminal goal tolerances
                tgx    = self._leader_goal.position.x if self._leader_goal else \
                          self._leader_path[-1].pose.position.x
                tgy    = self._leader_goal.position.y if self._leader_goal else \
                          self._leader_path[-1].pose.position.y
                tg_yaw = _yaw_from_quat(self._leader_goal.orientation) \
                          if self._leader_goal else \
                          _yaw_from_quat(self._leader_path[-1].pose.orientation)
                last_fp = self._follower_path[-1].pose

                l_dist = math.hypot(tgx - self._lx, tgy - self._ly)
                l_dyaw = abs(_angle_diff(tg_yaw, self._lyaw))
                f_dist = math.hypot(
                    last_fp.position.x - self._fx,
                    last_fp.position.y - self._fy)
                f_dyaw = abs(_angle_diff(
                    _yaw_from_quat(last_fp.orientation), self._fyaw))

                if (l_dist < self._goal_tol_xy and l_dyaw < self._goal_tol_yaw
                        and f_dist < self._goal_tol_xy
                        and f_dyaw < self._goal_tol_yaw):
                    self._goal_done = True
                    msg = Bool(); msg.data = True
                    self._reached_pub.publish(msg)
                    self.get_logger().info(
                        f"Formation goal reached!  "
                        f"L_dist={l_dist:.3f}m L_yaw={math.degrees(l_dyaw):.1f}deg  "
                        f"F_dist={f_dist:.3f}m F_yaw={math.degrees(f_dyaw):.1f}deg"
                    )

        # Publish status
        lp = self._leader_path[idx].pose
        fp = self._follower_path[idx].pose
        l_dist = math.hypot(lp.position.x - self._lx, lp.position.y - self._ly)
        f_dist = math.hypot(fp.position.x - self._fx, fp.position.y - self._fy)
        st = String()
        st.data = (
            f"wp={idx}/{n-1}  "
            f"L:({l_dist:.2f}m,{'Y' if l_ok else 'N'})  "
            f"F:({f_dist:.2f}m,{'Y' if f_ok else 'N'})"
        )
        self._status_pub.publish(st)


def main(args=None):
    rclpy.init(args=args)
    node = FormationSyncController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
