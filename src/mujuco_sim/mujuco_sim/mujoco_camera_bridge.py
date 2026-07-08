#!/usr/bin/env python3

import math
import mmap
import os
import struct

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import StaticTransformBroadcaster


SHM_PATH = "/dev/shm/unitree_mujoco_nav"
MAGIC = 0x564E4A4D
VERSION_WITH_IMU = 3
STRUCT_FORMAT_V3 = "<IIIIQd7d6d3d3d4d3d3d360f72000f"
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT_V3)


class MujocoCameraBridge(Node):
    def __init__(self):
        super().__init__("mujoco_camera_bridge")

        self.declare_parameter("model_path", "")
        self.declare_parameter("shm_path", SHM_PATH)
        self.declare_parameter("camera_name", "head_camera")
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_link_frame", "camera_link")
        self.declare_parameter("color_optical_frame", "camera_color_optical_frame")
        self.declare_parameter("depth_optical_frame", "camera_depth_optical_frame")
        self.declare_parameter("color_topic", "/camera/color/image_raw")
        self.declare_parameter("depth_topic", "/camera/depth/image_rect_raw")
        self.declare_parameter("color_info_topic", "/camera/color/camera_info")
        self.declare_parameter("depth_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("camera_xyz", "0.10,0.0,0.34")
        self.declare_parameter("horizontal_fov_deg", 69.4)

        self.model_path = self.get_parameter("model_path").value
        self.shm_path = self.get_parameter("shm_path").value
        self.camera_name = self.get_parameter("camera_name").value
        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.base_frame = self.get_parameter("base_frame").value
        self.camera_link_frame = self.get_parameter("camera_link_frame").value
        self.color_optical_frame = self.get_parameter("color_optical_frame").value
        self.depth_optical_frame = self.get_parameter("depth_optical_frame").value
        self.camera_xyz = self._parse_xyz(self.get_parameter("camera_xyz").value)
        self.horizontal_fov_deg = float(self.get_parameter("horizontal_fov_deg").value)

        self.color_pub = self.create_publisher(
            Image, self.get_parameter("color_topic").value, 5
        )
        self.depth_pub = self.create_publisher(
            Image, self.get_parameter("depth_topic").value, 5
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("color_info_topic").value, 5
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, self.get_parameter("depth_info_topic").value, 5
        )
        self.static_tf_pub = StaticTransformBroadcaster(self)

        self.fd = None
        self.mm = None
        self.seq = 0

        self._load_mujoco()
        self._publish_static_tf()

        fps = float(self.get_parameter("fps").value)
        self.create_timer(1.0 / fps, self._timer)
        self.get_logger().info(
            f"Publishing simulated RGB-D camera '{self.camera_name}' from {self.model_path}"
        )

    def _load_mujoco(self):
        try:
            import mujoco
        except ImportError as exc:
            raise RuntimeError(
                "Python package 'mujoco' is required for mujoco_camera_bridge. "
                "Install a version matching the simulator, e.g. `pip install mujoco==3.2.7`."
            ) from exc

        if not self.model_path or not os.path.exists(self.model_path):
            raise RuntimeError(f"MuJoCo model_path does not exist: {self.model_path}")

        self.mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        self.renderer = mujoco.Renderer(self.model, height=self.height, width=self.width)
        self.camera_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, self.camera_name)
        if self.camera_id < 0:
            raise RuntimeError(f"Camera '{self.camera_name}' not found in {self.model_path}")
        mujoco.mj_forward(self.model, self.data)

    def _parse_xyz(self, text):
        values = [float(item.strip()) for item in text.split(",") if item.strip()]
        if len(values) != 3:
            raise ValueError("camera_xyz must contain 3 comma-separated values")
        return values

    def _open_if_needed(self):
        if self.mm is not None:
            return True
        if not os.path.exists(self.shm_path):
            return False
        if os.path.getsize(self.shm_path) < STRUCT_SIZE:
            return False
        self.fd = os.open(self.shm_path, os.O_RDONLY)
        self.mm = mmap.mmap(self.fd, STRUCT_SIZE, access=mmap.ACCESS_READ)
        return True

    def _read_pose(self):
        if not self._open_if_needed():
            return None
        self.mm.seek(0)
        magic, version, _num_ranges, _num_points = struct.unpack("<IIII", self.mm.read(16))
        if magic != MAGIC or version < VERSION_WITH_IMU:
            return None
        self.mm.seek(0)
        data = struct.unpack(STRUCT_FORMAT_V3, self.mm.read(STRUCT_SIZE))
        return data[6:13]

    def _publish_static_tf(self):
        stamp = rclpy.time.Time().to_msg()
        transforms = []

        camera_link = TransformStamped()
        camera_link.header.stamp = stamp
        camera_link.header.frame_id = self.base_frame
        camera_link.child_frame_id = self.camera_link_frame
        camera_link.transform.translation.x = self.camera_xyz[0]
        camera_link.transform.translation.y = self.camera_xyz[1]
        camera_link.transform.translation.z = self.camera_xyz[2]
        camera_link.transform.rotation.w = 1.0
        transforms.append(camera_link)

        # base/camera_link uses ROS body axes: x forward, y left, z up.
        # Optical frames use REP-103 camera axes: z forward, x right, y down.
        optical_q = (-0.5, 0.5, -0.5, 0.5)
        for child in (self.color_optical_frame, self.depth_optical_frame):
            optical = TransformStamped()
            optical.header.stamp = stamp
            optical.header.frame_id = self.camera_link_frame
            optical.child_frame_id = child
            optical.transform.rotation.x = optical_q[0]
            optical.transform.rotation.y = optical_q[1]
            optical.transform.rotation.z = optical_q[2]
            optical.transform.rotation.w = optical_q[3]
            transforms.append(optical)

        self.static_tf_pub.sendTransform(transforms)

    def _camera_info(self, stamp, frame_id):
        msg = CameraInfo()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.width = self.width
        msg.height = self.height
        hfov = math.radians(self.horizontal_fov_deg)
        fx = self.width / (2.0 * math.tan(hfov / 2.0))
        fy = fx
        cx = (self.width - 1.0) / 2.0
        cy = (self.height - 1.0) / 2.0
        msg.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        msg.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        msg.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        msg.distortion_model = "plumb_bob"
        return msg

    def _timer(self):
        pose = self._read_pose()
        if pose is None:
            return

        qpos_count = min(7, self.model.nq)
        self.data.qpos[:qpos_count] = np.asarray(pose[:qpos_count], dtype=np.float64)
        self.mujoco.mj_forward(self.model, self.data)

        stamp = self.get_clock().now().to_msg()

        self.renderer.disable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_id)
        rgb = np.asarray(self.renderer.render(), dtype=np.uint8)

        self.renderer.enable_depth_rendering()
        self.renderer.update_scene(self.data, camera=self.camera_id)
        depth = np.asarray(self.renderer.render(), dtype=np.float32)
        self.renderer.disable_depth_rendering()

        self.seq += 1
        color_msg = Image()
        color_msg.header.stamp = stamp
        color_msg.header.frame_id = self.color_optical_frame
        color_msg.height = self.height
        color_msg.width = self.width
        color_msg.encoding = "rgb8"
        color_msg.is_bigendian = False
        color_msg.step = self.width * 3
        color_msg.data = rgb.tobytes()
        self.color_pub.publish(color_msg)

        depth_msg = Image()
        depth_msg.header.stamp = stamp
        depth_msg.header.frame_id = self.depth_optical_frame
        depth_msg.height = self.height
        depth_msg.width = self.width
        depth_msg.encoding = "32FC1"
        depth_msg.is_bigendian = False
        depth_msg.step = self.width * 4
        depth_msg.data = depth.tobytes()
        self.depth_pub.publish(depth_msg)

        self.color_info_pub.publish(self._camera_info(stamp, self.color_optical_frame))
        self.depth_info_pub.publish(self._camera_info(stamp, self.depth_optical_frame))

    def destroy_node(self):
        if self.mm is not None:
            self.mm.close()
        if self.fd is not None:
            os.close(self.fd)
        if hasattr(self, "renderer"):
            self.renderer.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MujocoCameraBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
