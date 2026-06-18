#!/usr/bin/env python3
"""
Point cloud registration bridge.

The staircase estimation node expects its input cloud to already be expressed
in the odometry/world frame (it does NOT do any TF lookup -- it just stacks the
incoming points and crops a box around the vehicle's odom-frame position, see
PointCloudHandler in staircase_estimation_robot_node.cpp).

A raw LiDAR (e.g. /rslidar_points in frame 'lidar_link') is therefore not usable
directly. This node looks up the TF from the cloud's frame to the target frame
(default 'odom') and republishes the transformed cloud so the estimation node
can consume it.

Params:
  input_topic   (str)  default '/rslidar_points'
  output_topic  (str)  default '/registered_point_cloud'
  target_frame  (str)  default 'odom'
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time

from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2 as pc2

from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf2_ros import LookupException, ExtrapolationException, ConnectivityException

import numpy as np

_OUT_FIELDS = [
    PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
]


def _quat_to_matrix(x, y, z, w):
    """Return a 3x3 rotation matrix from a quaternion."""
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - (yy + zz), xy - wz,         xz + wy],
        [xy + wz,         1.0 - (xx + zz), yz - wx],
        [xz - wy,         yz + wx,         1.0 - (xx + yy)],
    ])


class PointCloudRegistrationNode(Node):
    def __init__(self):
        super().__init__('pointcloud_registration_node')

        self.declare_parameter('input_topic', '/rslidar_points')
        self.declare_parameter('output_topic', '/registered_point_cloud')
        self.declare_parameter('target_frame', 'odom')

        self.input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.target_frame = self.get_parameter('target_frame').get_parameter_value().string_value

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.sub = self.create_subscription(
            PointCloud2, self.input_topic, self._cloud_cb, qos_profile_sensor_data)
        self.pub = self.create_publisher(
            PointCloud2, self.output_topic, qos_profile_sensor_data)

        self._warned = False
        self.get_logger().info(
            f"Registering '{self.input_topic}' -> '{self.output_topic}' in frame '{self.target_frame}'")

    def _lookup(self, source_frame, stamp):
        """Try the cloud's exact stamp, fall back to the latest available TF."""
        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame, source_frame, stamp, timeout=Duration(seconds=0.0))
        except (LookupException, ExtrapolationException, ConnectivityException):
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame, source_frame, Time())
            except (LookupException, ExtrapolationException, ConnectivityException) as exc:
                if not self._warned:
                    self.get_logger().warn(
                        f"No TF {self.target_frame} <- {source_frame} yet: {exc}")
                    self._warned = True
                return None

    def _cloud_cb(self, msg: PointCloud2):
        tf = self._lookup(msg.header.frame_id, msg.header.stamp)
        if tf is None:
            return
        self._warned = False

        # read_points (not read_points_numpy) so mixed-datatype clouds are OK
        # (rslidar mixes float32 xyz/intensity with uint16 ring + float64 timestamp).
        arr = pc2.read_points(
            msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True)
        if arr.shape[0] == 0:
            return

        xyz = np.column_stack(
            (arr['x'], arr['y'], arr['z'])).astype(np.float64)
        intensity = arr['intensity'].astype(np.float32)

        t = tf.transform.translation
        q = tf.transform.rotation
        rot = _quat_to_matrix(q.x, q.y, q.z, q.w)
        xyz_t = xyz @ rot.T + np.array([t.x, t.y, t.z])

        out = np.empty((xyz_t.shape[0], 4), dtype=np.float32)
        out[:, :3] = xyz_t.astype(np.float32)
        out[:, 3] = intensity

        header = msg.header
        header.frame_id = self.target_frame
        self.pub.publish(pc2.create_cloud(header, _OUT_FIELDS, out))


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudRegistrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
