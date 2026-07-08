#!/usr/bin/env python3
"""Follower formation controller for the two-G1 MuJoCo setup.

The leader is driven by Nav2 through /cmd_vel. This node keeps the second
robot at a fixed offset from the leader and publishes /follower/cmd_vel.
The follower motion still goes through the real g1_ctrl policy and the
follower low-level MuJoCo bridge; this node only supplies the velocity target.
"""
from __future__ import annotations

import math
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def _angle_diff(target: float, current: float) -> float:
    diff = target - current
    return math.atan2(math.sin(diff), math.cos(diff))


def _clamp(value: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, value))


class LeaderFollowerController(Node):
    def __init__(self) -> None:
        super().__init__("leader_follower_controller")

        self.declare_parameter("leader_odom_topic", "/odom")
        self.declare_parameter("follower_odom_topic", "/follower/odom")
        self.declare_parameter("follower_cmd_topic", "/follower/cmd_vel")

        # Target follower pose in the leader frame. Face-to-face means the
        # follower is in front of the leader and yaw-flipped by pi.
        self.declare_parameter("target_offset_x", 1.8)
        self.declare_parameter("target_offset_y", 0.0)
        self.declare_parameter("target_yaw_offset", math.pi)

        self.declare_parameter("max_vx_forward", 0.30)
        self.declare_parameter("max_vx_backward", 0.15)
        self.declare_parameter("max_vy", 0.20)
        self.declare_parameter("max_vyaw", 0.50)
        self.declare_parameter("kp_xy", 1.1)
        self.declare_parameter("kp_yaw", 1.5)
        self.declare_parameter("xy_deadband", 0.04)
        self.declare_parameter("yaw_deadband", 0.05)
        self.declare_parameter("odom_timeout", 0.75)
        self.declare_parameter("control_hz", 20.0)

        p = self.get_parameter
        self._offset_x = float(p("target_offset_x").value)
        self._offset_y = float(p("target_offset_y").value)
        self._yaw_offset = float(p("target_yaw_offset").value)
        self._max_vx_fwd = float(p("max_vx_forward").value)
        self._max_vx_bwd = float(p("max_vx_backward").value)
        self._max_vy = float(p("max_vy").value)
        self._max_w = float(p("max_vyaw").value)
        self._kp_xy = float(p("kp_xy").value)
        self._kp_yaw = float(p("kp_yaw").value)
        self._xy_deadband = float(p("xy_deadband").value)
        self._yaw_deadband = float(p("yaw_deadband").value)
        self._odom_timeout = float(p("odom_timeout").value)

        self._leader = None
        self._follower = None
        self._leader_stamp = 0.0
        self._follower_stamp = 0.0

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._cmd_pub = self.create_publisher(
            Twist, str(p("follower_cmd_topic").value), reliable
        )
        self.create_subscription(
            Odometry, str(p("leader_odom_topic").value), self._on_leader_odom, reliable
        )
        self.create_subscription(
            Odometry,
            str(p("follower_odom_topic").value),
            self._on_follower_odom,
            reliable,
        )

        hz = max(1.0, float(p("control_hz").value))
        self.create_timer(1.0 / hz, self._control_step)
        self.get_logger().info(
            "Leader-follower controller ready: offset=(%.2f, %.2f)"
            % (self._offset_x, self._offset_y)
        )

    def _on_leader_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._leader = (
            pose.position.x,
            pose.position.y,
            _yaw_from_quat(pose.orientation),
        )
        self._leader_stamp = time.monotonic()

    def _on_follower_odom(self, msg: Odometry) -> None:
        pose = msg.pose.pose
        self._follower = (
            pose.position.x,
            pose.position.y,
            _yaw_from_quat(pose.orientation),
        )
        self._follower_stamp = time.monotonic()

    def _control_step(self) -> None:
        now = time.monotonic()
        if (
            self._leader is None
            or self._follower is None
            or now - self._leader_stamp > self._odom_timeout
            or now - self._follower_stamp > self._odom_timeout
        ):
            self._publish_zero()
            return

        lx, ly, lyaw = self._leader
        fx, fy, fyaw = self._follower

        c = math.cos(lyaw)
        s = math.sin(lyaw)
        tx = lx + c * self._offset_x - s * self._offset_y
        ty = ly + s * self._offset_x + c * self._offset_y
        tyaw = lyaw + self._yaw_offset

        dx = tx - fx
        dy = ty - fy
        cf = math.cos(-fyaw)
        sf = math.sin(-fyaw)
        ex = cf * dx - sf * dy
        ey = sf * dx + cf * dy
        eyaw = _angle_diff(tyaw, fyaw)

        if math.hypot(dx, dy) < self._xy_deadband:
            ex = 0.0
            ey = 0.0
        if abs(eyaw) < self._yaw_deadband:
            eyaw = 0.0

        cmd = Twist()
        cmd.linear.x = _clamp(self._kp_xy * ex, -self._max_vx_bwd, self._max_vx_fwd)
        cmd.linear.y = _clamp(self._kp_xy * ey, -self._max_vy, self._max_vy)
        cmd.angular.z = _clamp(self._kp_yaw * eyaw, -self._max_w, self._max_w)
        self._cmd_pub.publish(cmd)

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = LeaderFollowerController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
