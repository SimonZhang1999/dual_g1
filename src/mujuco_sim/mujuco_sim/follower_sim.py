#!/usr/bin/env python3
"""Kinematic follower robot simulator.

Simulates a second omnidirectional robot (follower) by integrating
/follower/cmd_vel commands into a 2-D pose.  Publishes /follower/odom
and the TF odom→follower_base_link so the rest of the stack (joint MPC,
formation sync controller, RViz) can use it exactly like a real robot.

When a real second G1 is available, simply launch its shm_bridge under
the /follower namespace and remove this node from the launch file.

Initial pose: base_distance metres in front of the leader's first odom pose,
              facing the leader (yaw = leader_yaw + π).
"""
from __future__ import annotations

import ctypes
import math
import mmap
import os
import struct
import threading

import rclpy
from geometry_msgs.msg import TransformStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
import tf2_ros

# ── Follower SHM layout (must match shm_nav_bridge.h) ─────────────────────────
# struct UnitreeMujocoFollowerShmData { uint32 magic, seq; float x,y,z,qw,qx,qy,qz; float joints[29]; }
_FOLLOWER_SHM_NAME   = "/unitree_mujoco_follower"
_FOLLOWER_SHM_MAGIC  = 0x464F4C57  # 'FOLW'
_FOLLOWER_SHM_FORMAT = "=II7f29f"   # little-endian: magic, seq, x,y,z,qw,qx,qy,qz, 29 joints
_FOLLOWER_SHM_SIZE   = struct.calcsize(_FOLLOWER_SHM_FORMAT)


def _open_follower_shm():
    """Open or create the follower SHM segment.  Returns a writable mmap or None."""
    try:
        import posix_ipc  # type: ignore
        mem = posix_ipc.SharedMemory(
            _FOLLOWER_SHM_NAME, flags=posix_ipc.O_CREAT | posix_ipc.O_RDWR,
            size=_FOLLOWER_SHM_SIZE, mode=0o666)
        mm = mmap.mmap(mem.fd, _FOLLOWER_SHM_SIZE)
        mem.close_fd()
        return mm
    except Exception:
        pass
    # Fallback: use /dev/shm directly
    shm_path = "/dev/shm" + _FOLLOWER_SHM_NAME
    try:
        fd = os.open(shm_path, os.O_CREAT | os.O_RDWR, 0o666)
        os.ftruncate(fd, _FOLLOWER_SHM_SIZE)
        mm = mmap.mmap(fd, _FOLLOWER_SHM_SIZE)
        os.close(fd)
        return mm
    except Exception as e:
        print(f"[follower_sim] Could not open follower SHM: {e}")
        return None


def _yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class FollowerSim(Node):
    def __init__(self) -> None:
        super().__init__("follower_sim")

        self.declare_parameter("leader_odom_topic",   "/odom")
        self.declare_parameter("leader_pub_topic",    "/leader/odom")
        self.declare_parameter("follower_odom_topic", "/follower/odom")
        self.declare_parameter("follower_cmd_topic",  "/follower/cmd_vel")
        self.declare_parameter("base_distance",       1.80)
        self.declare_parameter("odom_frame",          "odom")
        self.declare_parameter("follower_frame",      "follower_base_link")
        self.declare_parameter("sim_dt",              0.02)

        p = self.get_parameter
        self._base_dist      = float(p("base_distance").value)
        self._odom_frame     = str(p("odom_frame").value)
        self._follower_frame = str(p("follower_frame").value)
        self._dt             = float(p("sim_dt").value)

        # robot state
        self._fx   = 0.0
        self._fy   = 0.0
        self._fyaw = 0.0
        # latest cmd_vel
        self._vx = 0.0
        self._vy = 0.0
        self._w  = 0.0
        self._initialised = False
        self._lock = threading.Lock()

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._leader_pub   = self.create_publisher(
            Odometry, str(p("leader_pub_topic").value), reliable)
        self._follower_pub = self.create_publisher(
            Odometry, str(p("follower_odom_topic").value), reliable)

        self.create_subscription(
            Odometry, str(p("leader_odom_topic").value),
            self._on_leader_odom, reliable)
        self.create_subscription(
            Twist, str(p("follower_cmd_topic").value),
            self._on_cmd_vel, reliable)

        self._tf_br = tf2_ros.TransformBroadcaster(self)
        self._timer = self.create_timer(self._dt, self._integrate)

        # Open follower SHM for MuJoCo teleport
        self._shm_mm   = _open_follower_shm()
        self._shm_seq  = 0
        if self._shm_mm:
            self.get_logger().info("Follower SHM opened for MuJoCo teleport")
        else:
            self.get_logger().warn("Follower SHM not available; MuJoCo ghost disabled")

        self.get_logger().info(
            f"Follower sim started  dist={self._base_dist:.2f}m in front of leader"
        )

    def _on_leader_odom(self, msg: Odometry) -> None:
        # Always re-publish as /leader/odom (for joint MPC to read)
        lmsg = Odometry()
        lmsg.header          = msg.header
        lmsg.header.frame_id = self._odom_frame
        lmsg.child_frame_id  = "base_link"
        lmsg.pose            = msg.pose
        lmsg.twist           = msg.twist
        self._leader_pub.publish(lmsg)

        if self._initialised:
            return
        # One-time initialisation: place follower in front of leader
        x   = msg.pose.pose.position.x
        y   = msg.pose.pose.position.y
        yaw = _yaw_from_quat(msg.pose.pose.orientation)
        with self._lock:
            self._fx   = x + self._base_dist * math.cos(yaw)
            self._fy   = y + self._base_dist * math.sin(yaw)
            self._fyaw = math.atan2(
                math.sin(yaw + math.pi), math.cos(yaw + math.pi))
            self._initialised = True
        self.get_logger().info(
            f"Follower initialised at ({self._fx:.2f}, {self._fy:.2f}) "
            f"yaw={math.degrees(self._fyaw):.1f}°"
        )

    def _on_cmd_vel(self, msg: Twist) -> None:
        with self._lock:
            self._vx = msg.linear.x
            self._vy = msg.linear.y
            self._w  = msg.angular.z

    def _integrate(self) -> None:
        with self._lock:
            if not self._initialised:
                return
            # omnidirectional kinematics: body → world frame
            c = math.cos(self._fyaw)
            s = math.sin(self._fyaw)
            self._fx   += (c * self._vx - s * self._vy) * self._dt
            self._fy   += (s * self._vx + c * self._vy) * self._dt
            self._fyaw += self._w * self._dt
            self._fyaw  = math.atan2(math.sin(self._fyaw), math.cos(self._fyaw))
            fx, fy, fyaw = self._fx, self._fy, self._fyaw
            vx, vy, w    = self._vx, self._vy, self._w

        now = self.get_clock().now().to_msg()

        # Publish /follower/odom
        odom = Odometry()
        odom.header.stamp            = now
        odom.header.frame_id         = self._odom_frame
        odom.child_frame_id          = self._follower_frame
        odom.pose.pose.position.x    = fx
        odom.pose.pose.position.y    = fy
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.z = math.sin(fyaw * 0.5)
        odom.pose.pose.orientation.w = math.cos(fyaw * 0.5)
        odom.twist.twist.linear.x    = vx
        odom.twist.twist.linear.y    = vy
        odom.twist.twist.angular.z   = w
        self._follower_pub.publish(odom)

        # Broadcast TF: odom → follower_base_link
        tf = TransformStamped()
        tf.header.stamp            = now
        tf.header.frame_id         = self._odom_frame
        tf.child_frame_id          = self._follower_frame
        tf.transform.translation.x = fx
        tf.transform.translation.y = fy
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = math.sin(fyaw * 0.5)
        tf.transform.rotation.w    = math.cos(fyaw * 0.5)
        self._tf_br.sendTransform(tf)

        # Write to follower SHM so MuJoCo can teleport the ghost robot
        self._write_follower_shm(fx, fy, fyaw)

    def _write_follower_shm(self, x: float, y: float, yaw: float) -> None:
        if self._shm_mm is None:
            return
        self._shm_seq += 1
        qw = math.cos(yaw * 0.5)
        qz = math.sin(yaw * 0.5)
        # pack: magic, seq, x, y, z(=0.793), qw, qx(=0), qy(=0), qz, 29 zero joints
        data = struct.pack(
            _FOLLOWER_SHM_FORMAT,
            _FOLLOWER_SHM_MAGIC,
            self._shm_seq,
            float(x), float(y), 0.793,
            float(qw), 0.0, 0.0, float(qz),
            *([0.0] * 29)  # joints set to 0; C++ uses its own standing defaults
        )
        self._shm_mm.seek(0)
        self._shm_mm.write(data)


def main(args=None):
    rclpy.init(args=args)
    node = FollowerSim()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
