#!/usr/bin/env python3
"""Publish a virtual /follower/odom that lags the real robot by a fixed distance.

The follower is placed `base_distance` metres in front of the leader
(in the leader's body frame). Its yaw is flipped 180 deg so the two robots
are visually face-to-face, matching the box-carrying topology expected by
the joint MPC.
"""
from __future__ import annotations

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class VirtualFollowerOdom(Node):
    def __init__(self) -> None:
        super().__init__("virtual_follower_odom")
        self.declare_parameter("leader_odom_topic", "/odom")
        self.declare_parameter("follower_odom_topic", "/follower/odom")
        self.declare_parameter("leader_pub_topic", "/leader/odom")
        self.declare_parameter("base_distance", 1.80)
        self.declare_parameter("follower_frame", "follower_base_link")
        self.declare_parameter("odom_frame", "odom")

        reliable = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )

        self._dist = float(self.get_parameter("base_distance").value)
        self._follower_frame = str(self.get_parameter("follower_frame").value)
        self._odom_frame = str(self.get_parameter("odom_frame").value)

        self._leader_pub = self.create_publisher(
            Odometry, str(self.get_parameter("leader_pub_topic").value), reliable
        )
        self._follower_pub = self.create_publisher(
            Odometry, str(self.get_parameter("follower_odom_topic").value), reliable
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter("leader_odom_topic").value),
            self._on_odom,
            reliable,
        )
        self.get_logger().info(
            f"Virtual follower: base_distance={self._dist:.2f}m "
            f"in front of leader, face-to-face yaw"
        )

    def _on_odom(self, msg: Odometry) -> None:
        # Re-publish as /leader/odom with consistent child_frame_id
        leader_msg = Odometry()
        leader_msg.header = msg.header
        leader_msg.header.frame_id = self._odom_frame
        leader_msg.child_frame_id = "base_link"
        leader_msg.pose = msg.pose
        leader_msg.twist = msg.twist
        self._leader_pub.publish(leader_msg)

        # Extract leader 2-D pose
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z),
        )

        # Place follower `base_distance` in front of the leader (leader faces +x)
        fx = x + self._dist * math.cos(yaw)
        fy = y + self._dist * math.sin(yaw)
        # Face-to-face: follower yaw = leader_yaw + pi
        fyaw = math.atan2(math.sin(yaw + math.pi), math.cos(yaw + math.pi))

        follower_msg = Odometry()
        follower_msg.header.stamp = msg.header.stamp
        follower_msg.header.frame_id = self._odom_frame
        follower_msg.child_frame_id = self._follower_frame
        follower_msg.pose.pose.position.x = fx
        follower_msg.pose.pose.position.y = fy
        follower_msg.pose.pose.position.z = 0.0
        follower_msg.pose.pose.orientation.z = math.sin(fyaw * 0.5)
        follower_msg.pose.pose.orientation.w = math.cos(fyaw * 0.5)
        # Follower twist is zero (virtual)
        self._follower_pub.publish(follower_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VirtualFollowerOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
