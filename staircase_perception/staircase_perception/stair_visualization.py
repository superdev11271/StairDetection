#!/usr/bin/env python3
from rclpy.node import Node
from rclpy.duration import Duration

from visualization_msgs.msg import MarkerArray, Marker
from staircase_msgs.msg import StaircaseMsg, StaircaseMeasurement
from geometry_msgs.msg import Point, PointStamped, TransformStamped
from scipy.spatial.transform import Rotation

import tf2_geometry_msgs
import numpy as np
from math import sqrt

try:
    from scipy.spatial import ConvexHull, QhullError
except Exception:  # pragma: no cover - scipy layout differences across versions
    ConvexHull = None
    QhullError = Exception

class StairViz(Node):
    def __init__(self):
        """
        Initializes the ROS 2 node, declares parameters, creates publishers and subscribers.
        """
        super().__init__('stair_viz_node')

        # Declare parameters to match the C++ node and launch file
        self.declare_parameter('staircase_perception_topics.staircase_estimates_topic', '/staircase_estimation_robot_node/staircase_estimates')
        self.declare_parameter('staircase_perception_topics.staircase_measurements_topic', 'staircase_estimation_robot_node/staircase_measurements')
        
        self.staircase_estimates_topic = self.get_parameter('staircase_perception_topics.staircase_estimates_topic').get_parameter_value().string_value
        self.staircase_measurements_topic = self.get_parameter('staircase_perception_topics.staircase_measurements_topic').get_parameter_value().string_value
        
        self.simulation = self.get_parameter('use_sim_time').get_parameter_value().bool_value

        # Marker lifetime [s]. Markers expire after this long, so only currently-detected staircases
        # stay visible (they are re-published every frame); stale detections disappear automatically.
        self.declare_parameter('marker_lifetime', 0.5)
        self.marker_lifetime = self.get_parameter('marker_lifetime').get_parameter_value().double_value

        # Create subscribers
        self.create_subscription(StaircaseMeasurement, self.staircase_measurements_topic, self.stairs_measurement_cb, 10)
        self.create_subscription(StaircaseMsg, self.staircase_estimates_topic, self.stairs_detected_cb, 10)

        # Create publishers with the requested namespace
        self.stair_measurement_array_pub = self.create_publisher(MarkerArray, "/staircase_viz_node/stair_measurements_markers", 10)
        self.stair_estimate_array_pub = self.create_publisher(MarkerArray, "/staircase_viz_node/stair_estimate_markers", 10)

        # Footprint topic: each detected staircase projected flat onto a single plane (height ignored),
        # plus its circumscribed (minimum-area) rectangle drawn as a LINE_STRIP.
        self.stair_footprint_array_pub = self.create_publisher(MarkerArray, "/staircase_viz_node/stair_footprint_markers", 10)

        # Class variables to store state
        self.max_stairs_estimate_id_map = {}
        self.max_stairs_measurement_id_map = {}
        self.max_footprint_step_id_map = {}

        # Wipe any markers left in RViz by a previous run (e.g. from an older build that used a
        # long marker lifetime). Published on a short one-shot timer so RViz has time to connect.
        self._startup_clear_count = 0
        self._startup_clear_timer = self.create_timer(0.2, self._clear_stale_markers)

        self.get_logger().info(f"Stair visualization node started. Simulation: {self.simulation}")

    def _clear_stale_markers(self):
        """Publish a DELETEALL to both marker topics a few times right after startup."""
        clear = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        clear.markers.append(delete_all)
        self.stair_estimate_array_pub.publish(clear)
        self.stair_measurement_array_pub.publish(clear)
        self.stair_footprint_array_pub.publish(clear)
        self._startup_clear_count += 1
        if self._startup_clear_count >= 5:
            self._startup_clear_timer.cancel()

    def stairs_detected_cb(self, msg: StaircaseMsg):
        """
        Callback for handling incoming StaircaseMsg (estimates) and publishing markers.
        """
        stair_id = msg.stair_id
        current_count = msg.stair_count

        # Get the previous number of markers for this ID, defaulting to 0
        old_count = self.max_stairs_estimate_id_map.get(stair_id, 0)
        
        stairs_marker_array = MarkerArray()
        
        # Add or update markers for the current steps
        for i in range(current_count):
            stairs_marker = Marker()
            stairs_marker.header.frame_id = msg.frame_id
            stairs_marker.header.stamp = self.get_clock().now().to_msg()
            stairs_marker.ns = f"stairs_{stair_id}"
            stairs_marker.id = (stair_id * 100) + i
            stairs_marker.action = Marker.ADD
            stairs_marker.type = Marker.CUBE
            stairs_marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()

            # Set color and scale
            stairs_marker.color.r = 1.0
            stairs_marker.color.g = 1.0
            stairs_marker.color.b = 1.0
            stairs_marker.color.a = 1.0

            # Get points
            start_p = msg.steps_start_p[i]
            end_p = msg.steps_end_p[i]

            # Calculate center and orientation
            center_x = (start_p.x + end_p.x) / 2.0
            center_y = (start_p.y + end_p.y) / 2.0
            center_z = (start_p.z + end_p.z) / 2.0

            stairs_orient = np.arctan2((end_p.y - start_p.y), (end_p.x - start_p.x))
            
            rot = Rotation.from_euler('xyz', angles=[0, 0, stairs_orient])
            quat = rot.as_quat()
                
            stairs_marker.pose.position.x = center_x
            stairs_marker.pose.position.y = center_y
            stairs_marker.pose.position.z = center_z
            stairs_marker.pose.orientation.x = quat[0]
            stairs_marker.pose.orientation.y = quat[1]
            stairs_marker.pose.orientation.z = quat[2]
            stairs_marker.pose.orientation.w = quat[3]

            stairs_marker.scale.x = sqrt((end_p.x - start_p.x)**2 + (end_p.y - start_p.y)**2 + (end_p.z - start_p.z)**2)
            stairs_marker.scale.y = 0.05
            stairs_marker.scale.z = 0.05
            
            stairs_marker_array.markers.append(stairs_marker)

        # Delete old markers if the number of stairs has decreased
        if old_count > current_count:
            for i in range(current_count, old_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = msg.frame_id
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = f"stairs_{stair_id}"
                delete_marker.id = (stair_id * 100) + i
                delete_marker.action = Marker.DELETE
                stairs_marker_array.markers.append(delete_marker)

        self.stair_estimate_array_pub.publish(stairs_marker_array)
        self.max_stairs_estimate_id_map[stair_id] = current_count

        # Also emit the flattened footprint + circumscribed rectangle for this detection.
        self._publish_footprint(msg)

    def _min_area_rect(self, pts):
        """Return the 4 corners (4x2 array) of the minimum-area rectangle enclosing the 2D
        points, using rotating calipers on the convex hull. Falls back to an axis-aligned
        bounding box for degenerate input (few / collinear points)."""
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if pts.shape[0] == 0:
            return None
        if pts.shape[0] < 3 or ConvexHull is None:
            return self._aabb_corners(pts)
        try:
            hull = pts[ConvexHull(pts).vertices]
        except QhullError:
            return self._aabb_corners(pts)  # collinear / coincident points

        best_area = None
        best_corners = None
        n = len(hull)
        for i in range(n):
            edge = hull[(i + 1) % n] - hull[i]
            norm = float(np.hypot(edge[0], edge[1]))
            if norm < 1e-9:
                continue
            ux = edge / norm                       # one rectangle axis = this hull edge
            uy = np.array([-ux[1], ux[0]])         # perpendicular axis
            px = hull @ ux
            py = hull @ uy
            min_x, max_x = px.min(), px.max()
            min_y, max_y = py.min(), py.max()
            area = (max_x - min_x) * (max_y - min_y)
            if best_area is None or area < best_area:
                best_area = area
                local = np.array([[min_x, min_y], [max_x, min_y],
                                  [max_x, max_y], [min_x, max_y]])
                # Map corners from (ux, uy) basis back into world XY.
                best_corners = local @ np.vstack([ux, uy])
        if best_corners is None:
            return self._aabb_corners(pts)
        return best_corners

    @staticmethod
    def _aabb_corners(pts):
        pts = np.asarray(pts, dtype=float).reshape(-1, 2)
        if pts.shape[0] == 0:
            return None
        min_xy = pts.min(axis=0)
        max_xy = pts.max(axis=0)
        return np.array([[min_xy[0], min_xy[1]], [max_xy[0], min_xy[1]],
                         [max_xy[0], max_xy[1]], [min_xy[0], max_xy[1]]])

    def _publish_footprint(self, msg: StaircaseMsg):
        """Project every step of this staircase onto a single plane (height ignored),
        publish each projected step as a marker, then publish the circumscribed rectangle
        of all projected points as a closed LINE_STRIP -- on a separate topic."""
        stair_id = msg.stair_id
        current_count = msg.stair_count

        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        if current_count > 0:
            # Gather all endpoints and pick the plane height = lowest point (the base).
            pts2d = []
            z_plane = float('inf')
            for i in range(current_count):
                sp, ep = msg.steps_start_p[i], msg.steps_end_p[i]
                pts2d.append((sp.x, sp.y))
                pts2d.append((ep.x, ep.y))
                z_plane = min(z_plane, sp.z, ep.z)
            if not np.isfinite(z_plane):
                z_plane = 0.0

            # 1) The detection results, one marker per step, projected onto the plane.
            for i in range(current_count):
                sp, ep = msg.steps_start_p[i], msg.steps_end_p[i]
                step = Marker()
                step.header.frame_id = msg.frame_id
                step.header.stamp = now
                step.ns = f"footprint_steps_{stair_id}"
                step.id = (stair_id * 100) + i
                step.action = Marker.ADD
                step.type = Marker.LINE_STRIP
                step.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
                step.scale.x = 0.02
                step.color.r, step.color.g, step.color.b, step.color.a = 1.0, 1.0, 0.0, 0.8
                step.points = [Point(x=sp.x, y=sp.y, z=z_plane),
                               Point(x=ep.x, y=ep.y, z=z_plane)]
                marker_array.markers.append(step)

            # 2) Circumscribed rectangle of all projected points, as a closed LINE_STRIP.
            corners = self._min_area_rect(np.array(pts2d))
            if corners is not None:
                rect = Marker()
                rect.header.frame_id = msg.frame_id
                rect.header.stamp = now
                rect.ns = f"footprint_rect_{stair_id}"
                rect.id = stair_id
                rect.action = Marker.ADD
                rect.type = Marker.LINE_STRIP
                rect.lifetime = Duration(seconds=self.marker_lifetime).to_msg()
                rect.scale.x = 0.04
                rect.color.r, rect.color.g, rect.color.b, rect.color.a = 0.0, 1.0, 0.0, 1.0
                rect.points = [Point(x=float(c[0]), y=float(c[1]), z=z_plane) for c in corners]
                rect.points.append(rect.points[0])  # close the loop back to the first corner
                marker_array.markers.append(rect)

        # Delete stale per-step markers if the staircase shrank since last time.
        old_count = self.max_footprint_step_id_map.get(stair_id, 0)
        if old_count > current_count:
            for i in range(current_count, old_count):
                dm = Marker()
                dm.header.frame_id = msg.frame_id
                dm.header.stamp = now
                dm.ns = f"footprint_steps_{stair_id}"
                dm.id = (stair_id * 100) + i
                dm.action = Marker.DELETE
                marker_array.markers.append(dm)
        self.max_footprint_step_id_map[stair_id] = current_count

        self.stair_footprint_array_pub.publish(marker_array)

    def stairs_measurement_cb(self, msg: StaircaseMeasurement):
        """
        Callback for handling incoming StaircaseMeasurement and publishing markers.
        """
        stair_id = 1 if msg.is_ascending else 2
        current_count = msg.stair_count

        old_count = self.max_stairs_measurement_id_map.get(stair_id, 0)

        stairs_marker_array = MarkerArray()
        
        transform_stamped = TransformStamped()
        transform_stamped.transform = msg.robot_transform
        
        for i in range(current_count):
            stairs_marker = Marker()
            stairs_marker.header.frame_id = msg.frame_id
            stairs_marker.header.stamp = self.get_clock().now().to_msg()
            stairs_marker.ns = f"measurement_cube_{stair_id}"
            stairs_marker.id = (stair_id * 100) + i
            stairs_marker.action = Marker.ADD
            stairs_marker.type = Marker.CUBE
            stairs_marker.lifetime = Duration(seconds=self.marker_lifetime).to_msg()

            stairs_marker.color.r = 0.028
            stairs_marker.color.g = 0.681
            stairs_marker.color.b = 0.960
            stairs_marker.color.a = 1.0
            
            # Create PointStamped messages to use tf2_geometry_msgs
            start_p_stamped = PointStamped()
            start_p_stamped.point = msg.steps_start_p[i]
            start_p_transformed = tf2_geometry_msgs.do_transform_point(start_p_stamped, transform_stamped)

            end_p_stamped = PointStamped()
            end_p_stamped.point = msg.steps_end_p[i]
            end_p_transformed = tf2_geometry_msgs.do_transform_point(end_p_stamped, transform_stamped)
            
            center_x = (start_p_transformed.point.x + end_p_transformed.point.x) / 2.0
            center_y = (start_p_transformed.point.y + end_p_transformed.point.y) / 2.0
            
            stairs_orient = np.arctan2((end_p_transformed.point.y - start_p_transformed.point.y), (end_p_transformed.point.x - start_p_transformed.point.x))
            rot = Rotation.from_euler('xyz', angles=[0, 0, stairs_orient])
            quat = rot.as_quat()
            
            # Calculate height and depth relative to other steps
            if i == current_count - 1 and current_count > 1:
                height = abs(msg.steps_start_p[i].z - msg.steps_start_p[i-1].z)
                depth = sqrt((msg.steps_start_p[i].x - msg.steps_start_p[i-1].x)**2 + (msg.steps_start_p[i].y - msg.steps_start_p[i-1].y)**2)
            elif current_count > 1:
                height = abs(msg.steps_start_p[i+1].z - msg.steps_start_p[i].z)
                depth = sqrt((msg.steps_start_p[i+1].x - msg.steps_start_p[i].x)**2 + (msg.steps_start_p[i+1].y - msg.steps_start_p[i].y)**2)
            else: # Handle case with only one step
                height = 0.1 
                depth = 0.2
            
            center_z = (start_p_transformed.point.z + end_p_transformed.point.z) / 2.0

            stairs_marker.pose.position.x = center_x
            stairs_marker.pose.position.y = center_y
            stairs_marker.pose.position.z = center_z - height / 2.0 # Position at the base of the tread
            stairs_marker.pose.orientation.x = quat[0]
            stairs_marker.pose.orientation.y = quat[1]
            stairs_marker.pose.orientation.z = quat[2]
            stairs_marker.pose.orientation.w = quat[3]

            stairs_marker.scale.x = msg.step_lengths[i]
            stairs_marker.scale.y = depth
            stairs_marker.scale.z = height
            
            stairs_marker_array.markers.append(stairs_marker)

        # Delete old markers
        if old_count > current_count:
            for i in range(current_count, old_count):
                delete_marker = Marker()
                delete_marker.header.frame_id = msg.frame_id
                delete_marker.header.stamp = self.get_clock().now().to_msg()
                delete_marker.ns = f"measurement_cube_{stair_id}"
                delete_marker.id = (stair_id * 100) + i
                delete_marker.action = Marker.DELETE
                stairs_marker_array.markers.append(delete_marker)

        self.stair_measurement_array_pub.publish(stairs_marker_array)
        self.max_stairs_measurement_id_map[stair_id] = current_count
