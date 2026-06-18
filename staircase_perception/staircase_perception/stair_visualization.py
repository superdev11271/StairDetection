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

        # Class variables to store state
        self.max_stairs_estimate_id_map = {}
        self.max_stairs_measurement_id_map = {}

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
