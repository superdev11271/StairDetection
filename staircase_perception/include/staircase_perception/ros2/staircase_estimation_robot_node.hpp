#ifndef _STAIRCASE_ESTIMATION_ROBOT_NODE_H_
#define _STAIRCASE_ESTIMATION_ROBOT_NODE_H_

// ROS 2 Headers
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/bool.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

// TF2 Headers
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include <tf2/transform_datatypes.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/convert.h"

// PCL Headers
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include "pcl_ros/transforms.hpp"
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/io/pcd_io.h>

#include "staircase_msgs/msg/staircase_measurement.hpp"
#include "staircase_msgs/msg/staircase_msg.hpp"

// Project-specific Headers
#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/core/stair_detector.hpp"
#include "staircase_perception/core/multi_stair_manager.hpp"

// C++ Standard Libraries
#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdlib.h>
#include <chrono>
#include <unordered_map>
#include <limits>
#include <cmath>

// Using statements for easier access to message types
using std::placeholders::_1;

class StaircaseEstimationRobotNode : public rclcpp::Node
{
    public:
        StaircaseEstimationRobotNode();

    private:
        // ROS 2 Subscribers
        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr enable_staircase_sub_;
        rclcpp::Subscription<geometry_msgs::msg::TransformStamped>::SharedPtr global_tf_sub_;

        // ROS 2 Publishers
        rclcpp::Publisher<staircase_msgs::msg::StaircaseMsg>::SharedPtr stairs_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr staircase_status_pub_;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr robot_on_staircase_pub_;
        rclcpp::Publisher<staircase_msgs::msg::StaircaseMeasurement>::SharedPtr measurement_pub_;
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr process_cl_pub1_; 
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr process_cl_pub2_; 
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr line_marker_pub_;


        // ROS 2 Timers
        rclcpp::TimerBase::SharedPtr detection_timer_;

        // Main perception loop
        void PerceiveStaircases();

        // Publishing methods
        void publishStaircaseMeasurement(const stair_utility::StaircaseMeasurement& measurement, bool ascending);
        void publishStaircaseEstimate(const stair_utility::StaircaseEstimate& estimate);
        void publishLineMarkers();

        // When the detector finds nothing this frame, re-publish recently-detected staircases that
        // are still within detection_retention_range_ of the robot (bridges momentary dropouts).
        void republishRecentStaircases();

        // True when the robot's (x, y) lies inside the circumscribed rectangle of any known
        // staircase footprint (height ignored) and its z lies within that staircase's z-span.
        bool isRobotOnStaircase() const;
        // Publish, every cycle, the boolean result of isRobotOnStaircase() on robot_on_staircase_topic_.
        void publishRobotOnStaircase();

        // Callback methods
        void HandleEnable(const std_msgs::msg::Bool::SharedPtr msg);
        void OdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg);
        void PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        void GlobalTFHandler(const geometry_msgs::msg::TransformStamped::SharedPtr msg);

        // Helper method
        inline void averagePoint(const geometry_msgs::msg::Point& A, const geometry_msgs::msg::Point& B, geometry_msgs::msg::Point& C)
        {
            C.x = (A.x + B.x) / 2;
            C.y = (A.y + B.y) / 2;
            C.z = (A.z + B.z) / 2;
        }

        // Member Variables
        // Topic Names
        std::string pointcloud_topic_, odom_topic_, staircase_node_status_topic_, staircase_node_toggle_topic_, global_tf_topic_;
        std::string staircase_estimates_topic_, staircase_measurements_topic_, robot_on_staircase_topic_;

        // Robot-on-staircase test tolerances: the robot must be within on_staircase_proximity_ [m]
        // (3D, to an actual step) AND inside the footprint rectangle expanded by on_staircase_xy_margin_ [m].
        double on_staircase_xy_margin_, on_staircase_proximity_;

        // ROSNode Variables
        std::string global_frame_id_, body_frame_id_, odom_frame_id_, robot_name_;

        double ros_rate_; // This is used to set the timer frequency
        bool debug_, enable_stair_detection_, publish_staircase_measurement_, transform_detections_to_global_, simulation_;
        double debug_marker_lifetime_; // [s] lifetime of debug line markers so stale ones decay instead of freezing
        std::string robots_topics_prefix_;
        
        rclcpp::Time last_heartbeat_publish_time_;
        std_msgs::msg::Bool staircase_heartbeat_msg_;

        // Odometry Variables
        double vehicle_roll_, vehicle_pitch_, vehicle_yaw_;
        double vehicle_x_, vehicle_y_, vehicle_z_;
        geometry_msgs::msg::Pose vehicle_pose_;
        stair_utility::RobotPositionInfo robot_pos_;

        // Transform Variables (using tf2 types)
        geometry_msgs::msg::TransformStamped global_to_odom_tf_stamped_;
        tf2::Transform robot_pose_transform_; // tf2::Transform for calculations
        tf2::Transform global_tf_;         
        tf2::Transform odom_to_robot_tf_, global_to_odom_tf_;  

        // Lidar Variables
        rclcpp::Time lasercloud_time_;
        double min_range_x_, max_range_x_, min_range_y_, max_range_y_, min_range_z_, max_range_z_;
        double leaf_size_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_stacked_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_local_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_cropped_;

        bool new_lasercloud_, odom_initialized_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_processed_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_processed2_;

        // PCL Cloud Processing Objects
        pcl::VoxelGrid<pcl::PointXYZI> voxel_grid_filter_;
        pcl::CropBox<pcl::PointXYZI> box_filter_;
        double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_; // Variables for Box filter

        // Staircase Detector Objects and Parameters
        stair_utility::LineExtractorParams line_params_;
        stair_utility::StaircaseDetectorParams detector_params_;
        StairDetector detector_;

        stair_utility::StaircaseDetectorResult stair_detected_;
        stair_utility::StaircaseMeasurement staircase_up_, staircase_down_;

        // Staircase Manager Params and Objects
        stair_utility::StairManagerParams stair_manager_params_;
        SingleRobotStairManager manager_;

        stair_utility::StaircaseEstimate stair_up_estimate_, stair_down_estimate_;

        // Cache of the most recent published estimate per staircase id, used to keep nearby
        // staircases visible during frames where the detector returns nothing.
        std::unordered_map<int, stair_utility::StaircaseEstimate> last_published_estimates_;
        double detection_retention_range_; // [m] keep re-publishing cached staircases within this range of the robot

        // Time of the most recent *actual* detection per staircase id. Retention only re-publishes for
        // detection_hold_time_ seconds after this, so a staircase the robot can no longer see stops being
        // shown instead of lingering just because the robot is still nearby. <= 0 means no time limit.
        std::unordered_map<int, rclcpp::Time> last_detection_time_;
        double detection_hold_time_;
};

#endif // _STAIRCASE_ROBOTNODE_H_
