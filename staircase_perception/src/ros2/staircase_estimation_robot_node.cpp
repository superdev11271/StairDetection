#include "staircase_perception/ros2/staircase_estimation_robot_node.hpp"

// Constructor for the ROS 2 Node
StaircaseEstimationRobotNode::StaircaseEstimationRobotNode() : Node("staircase_estimation_robot_node")
{
    // === 1. Declare and Get Parameters ===    

    this->declare_parameter<std::string>("staircase_perception_topics.point_cloud_topic", "registered_point_cloud");
    this->declare_parameter<std::string>("staircase_perception_topics.odometry_topic", "odometry");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_status_topic", "staircase_estimation_robot_node/status");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_enable_topic", "staircase_estimation_robot_node/enable");
    this->declare_parameter<std::string>("staircase_perception_topics.global_transform_topic", "transform");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_estimates_topic", "staircase_estimation_robot_node/staircase_estimates");
    this->declare_parameter<std::string>("staircase_perception_topics.staircase_measurements_topic", "staircase_estimation_robot_node/staircase_measurements");


    this->get_parameter("staircase_perception_topics.point_cloud_topic", pointcloud_topic_);
    this->get_parameter("staircase_perception_topics.odometry_topic", odom_topic_);
    this->get_parameter("staircase_perception_topics.staircase_status_topic", staircase_node_status_topic_);
    this->get_parameter("staircase_perception_topics.staircase_enable_topic", staircase_node_toggle_topic_);
    this->get_parameter("staircase_perception_topics.global_transform_topic", global_tf_topic_);
    this->get_parameter("staircase_perception_topics.staircase_estimates_topic", staircase_estimates_topic_);
    this->get_parameter("staircase_perception_topics.staircase_measurements_topic", staircase_measurements_topic_);


    // ROS Node parameters
    this->declare_parameter<std::string>("staircase_perception_params.global_frame_id", "global");

    this->get_parameter("staircase_perception_params.global_frame_id", global_frame_id_);
    this->get_parameter("use_sim_time", simulation_);

    this->declare_parameter<double>("ros_rate", 10.0);
    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<std::string>("body_frame_id", "base_link");
    this->declare_parameter<std::string>("odom_frame_id", "odom");
    this->declare_parameter<std::string>("robot_name", "robot");
    this->declare_parameter<bool>("publish_measurements", false);
    this->declare_parameter<bool>("transform_detections_to_global", true);
    this->declare_parameter<std::string>("robot_topics_prefix", "");
    this->declare_parameter<double>("detection_retention_range", 5.0);
    this->declare_parameter<double>("detection_hold_time", 1.0);
    this->declare_parameter<double>("debug_marker_lifetime", 0.5);

    this->get_parameter("ros_rate", ros_rate_);
    this->get_parameter("debug", debug_);
    this->get_parameter("body_frame_id", body_frame_id_);
    this->get_parameter("odom_frame_id", odom_frame_id_);
    this->get_parameter("robot_name", robot_name_);
    this->get_parameter("publish_measurements", publish_staircase_measurement_);
    this->get_parameter("transform_detections_to_global", transform_detections_to_global_);
    this->get_parameter("robot_topics_prefix", robots_topics_prefix_);
    this->get_parameter("detection_retention_range", detection_retention_range_);
    this->get_parameter("detection_hold_time", detection_hold_time_);
    this->get_parameter("debug_marker_lifetime", debug_marker_lifetime_);

    // Point cloud pre-processing params
    this->declare_parameter<double>("stair_pointcloud.leaf_size", 0.025);
    this->declare_parameter<double>("stair_pointcloud.min_range_y", -6.0);
    this->declare_parameter<double>("stair_pointcloud.max_range_y", 6.0);
    this->declare_parameter<double>("stair_pointcloud.min_range_x", -6.0);
    this->declare_parameter<double>("stair_pointcloud.max_range_x", 6.0);
    this->declare_parameter<double>("stair_pointcloud.min_range_z", -2.5);
    this->declare_parameter<double>("stair_pointcloud.max_range_z", 1.5);

    this->get_parameter("stair_pointcloud.leaf_size", leaf_size_);
    this->get_parameter("stair_pointcloud.min_range_y", min_range_y_);
    this->get_parameter("stair_pointcloud.max_range_y", max_range_y_);
    this->get_parameter("stair_pointcloud.min_range_x", min_range_x_);
    this->get_parameter("stair_pointcloud.max_range_x", max_range_x_);
    this->get_parameter("stair_pointcloud.min_range_z", min_range_z_);
    this->get_parameter("stair_pointcloud.max_range_z", max_range_z_);

    // Staircase Detector Parameters
    this->declare_parameter<double>("stair_detector.angle_resolution", 1.0);
    this->declare_parameter<double>("stair_detector.robot_height", 0.75);
    this->declare_parameter<int>("stair_detector.min_stair_count", 3.0);
    this->declare_parameter<double>("stair_detector.stair_slope_min", 0.35);
    this->declare_parameter<double>("stair_detector.stair_slope_max", 1.22);
    this->declare_parameter<bool>("stair_detector.use_ramp_detection", true);
    this->declare_parameter<double>("stair_detector.initialization_range", 0.5);
    this->declare_parameter<double>("stair_detector.ground_height_buffer", 0.05);

    this->declare_parameter<double>("stair_detector.min_stair_width", 0.75);
    this->declare_parameter<double>("stair_detector.min_stair_height", 0.1);
    this->declare_parameter<double>("stair_detector.max_stair_height", 0.25);
    this->declare_parameter<double>("stair_detector.mix_stair_depth", 0.125);
    this->declare_parameter<double>("stair_detector.max_stair_depth", 0.35);
    this->declare_parameter<double>("stair_detector.max_stair_curvature", 0.55);

    // Detection gating (<= 0.0 disables a gate)
    this->declare_parameter<double>("stair_detector.max_detection_range", -1.0);
    this->declare_parameter<double>("stair_detector.max_line_fit_stddev", -1.0);
    this->declare_parameter<double>("stair_detector.max_step_depth_variation", -1.0);
    this->declare_parameter<double>("stair_detector.max_step_height_variation", -1.0);

    this->get_parameter("stair_detector.angle_resolution", detector_params_.angle_resolution);
    this->get_parameter("stair_detector.robot_height", detector_params_.robot_height);
    this->get_parameter("stair_detector.min_stair_count", detector_params_.min_stair_count);
    this->get_parameter("stair_detector.stair_slope_min", detector_params_.stair_slope_min);
    this->get_parameter("stair_detector.stair_slope_max", detector_params_.stair_slope_max);
    this->get_parameter("stair_detector.use_ramp_detection", detector_params_.use_ramp_detection);
    this->get_parameter("stair_detector.initialization_range", detector_params_.initialization_range);
    this->get_parameter("stair_detector.ground_height_buffer", detector_params_.ground_height_buffer);

    this->get_parameter<double>("stair_detector.min_stair_width", detector_params_.min_stair_width);
    this->get_parameter<double>("stair_detector.min_stair_height", detector_params_.min_stair_height);
    this->get_parameter<double>("stair_detector.max_stair_height", detector_params_.max_stair_height);
    this->get_parameter<double>("stair_detector.mix_stair_depth", detector_params_.min_stair_depth);
    this->get_parameter<double>("stair_detector.max_stair_depth", detector_params_.max_stair_depth);
    this->get_parameter<double>("stair_detector.max_stair_curvature", detector_params_.max_stair_curvature);

    this->get_parameter<double>("stair_detector.max_detection_range", detector_params_.max_detection_range);
    this->get_parameter<double>("stair_detector.max_line_fit_stddev", detector_params_.max_line_fit_stddev);
    this->get_parameter<double>("stair_detector.max_step_depth_variation", detector_params_.max_step_depth_variation);
    this->get_parameter<double>("stair_detector.max_step_height_variation", detector_params_.max_step_height_variation);

    detector_params_.leaf_size = leaf_size_;
    detector_params_.x_max = max_range_x_;
    detector_params_.x_min = min_range_x_;
    detector_params_.y_max = max_range_y_;
    detector_params_.y_min = min_range_y_;
    detector_params_.z_max = max_range_z_;
    detector_params_.z_min = min_range_z_;
    
    // Line Extractor parameters
    this->declare_parameter<double>("stair_line_extractor.bearing_variance", 0.0001);
    this->declare_parameter<double>("stair_line_extractor.range_variance", 0.001);
    this->declare_parameter<double>("stair_line_extractor.z_variance", 0.0004);
    this->declare_parameter<double>("stair_line_extractor.least_sq_angle_thresh", 0.05);
    this->declare_parameter<double>("stair_line_extractor.least_sq_radius_thresh", 0.075);
    this->declare_parameter<double>("stair_line_extractor.max_line_gap", 0.2);
    this->declare_parameter<double>("stair_line_extractor.min_range", 0.1);
    this->declare_parameter<double>("stair_line_extractor.max_range", 5.0);
    this->declare_parameter<double>("stair_line_extractor.min_split_distance", 0.2);
    this->declare_parameter<double>("stair_line_extractor.outlier_distance", 0.2);
    this->declare_parameter<int>("stair_line_extractor.min_line_points", 7);

    int min_line_points_int;
    this->get_parameter("stair_line_extractor.bearing_variance", line_params_.bearing_var);
    this->get_parameter("stair_line_extractor.range_variance", line_params_.range_var);
    this->get_parameter("stair_line_extractor.z_variance", line_params_.z_var);
    this->get_parameter("stair_line_extractor.least_sq_angle_thresh", line_params_.least_sq_angle_thresh);
    this->get_parameter("stair_line_extractor.least_sq_radius_thresh", line_params_.least_sq_radius_thresh);
    this->get_parameter("stair_line_extractor.max_line_gap", line_params_.max_line_gap);
    this->get_parameter("stair_line_extractor.min_range", line_params_.min_range);
    this->get_parameter("stair_line_extractor.max_range", line_params_.max_range);
    this->get_parameter("stair_line_extractor.min_split_distance", line_params_.min_split_dist);
    this->get_parameter("stair_line_extractor.outlier_distance", line_params_.outlier_dist);
    this->get_parameter("stair_line_extractor.min_line_points", min_line_points_int);
    line_params_.min_line_points = min_line_points_int;

    // Staircase Manager parameters
    this->declare_parameter<double>("stair_manager.yaw_threshold", 0.75);
    this->declare_parameter<std::string>("stair_manager.filter_type", "l_ekf");
    this->declare_parameter<double>("stair_manager.max_surface_z_threshold", 0.05);
    this->declare_parameter<int>("stair_manager.min_detections_to_confirm", 1);

    std::string filter_type;
    this->get_parameter("stair_manager.yaw_threshold", stair_manager_params_.yaw_threshold);
    this->get_parameter("stair_manager.filter_type", filter_type);
    this->get_parameter("stair_manager.min_detections_to_confirm", stair_manager_params_.min_detections_to_confirm);
    stair_manager_params_.robot_name = robot_name_;

    // === 2. Initialize Member Variables and Objects ===
    // This section is largely the same, just initializing variables.

    lasercloud_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_local_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_stacked_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_cropped_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_processed_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_processed2_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    lasercloud_->reserve(300000);
    long max_points_in_voxel_grid = round(((max_range_x_ - min_range_x_) / leaf_size_) * ((max_range_y_ - min_range_y_) / leaf_size_) * ((max_range_z_ - min_range_z_) / leaf_size_));
    lasercloud_stacked_->reserve(max_points_in_voxel_grid);
    lasercloud_cropped_->reserve(max_points_in_voxel_grid);
    lasercloud_local_->reserve(max_points_in_voxel_grid);

    vehicle_roll_ = 0;
    vehicle_pitch_ = 0;
    vehicle_yaw_ = 0;
    vehicle_x_ = 0;
    vehicle_y_ = 0;
    vehicle_z_ = 0;

    new_lasercloud_ = false;
    odom_initialized_ = false;
    enable_stair_detection_ = true;
    last_heartbeat_publish_time_ = this->now();
    
    global_to_odom_tf_stamped_.transform.rotation.w = 1.0; // Identity transform
    global_to_odom_tf_.setIdentity();
    
    // Set parameters for the Staircase Detector and the Line Extractor
    detector_ = StairDetector(detector_params_, line_params_);

    if (filter_type == "averg") {
        stair_manager_params_.filter_type = stair_utility::StaircaseFilterType::SimpleAveraging;
        RCLCPP_INFO(this->get_logger(), "\033[1;35m [Staircase Robot Node] Using basic averaging for merging stairs! \033[0m");
    } else if (filter_type == "maxim") {
        stair_manager_params_.filter_type = stair_utility::StaircaseFilterType::SimpleMaximum;
        RCLCPP_INFO(this->get_logger(), "\033[1;35m [Staircase Robot Node] Using simple maximizing function for merging stairs! \033[0m");
    } else if (filter_type == "l_ekf") {
        // This part needs to declare all the nested EKF parameters
        this->declare_parameter<std::vector<double>>("stair_ekf_params.initial_measurement_sigmas", {0.1, 0.1, 0.2, 0.2});
        this->declare_parameter<std::vector<double>>("stair_ekf_params.initial_pose_sigmas", {0.00, 0.00, 0.005, 0.03});
        this->declare_parameter<std::vector<double>>("stair_ekf_params.measurement_sigmas", {0.075, 0.075, 0.07, 0.071});
        this->declare_parameter<std::vector<double>>("stair_ekf_params.pose_sigmas", {0.000, 0.00, 0.02, 0.03});
        this->declare_parameter<std::vector<double>>("stair_ekf_params.model_noise_sigmas", {0.005, 0.005, 0.0, 0.05, 0.1, 0.05});

        std::vector<double> params_vec;
        this->get_parameter("stair_ekf_params.initial_measurement_sigmas", params_vec);
        stair_manager_params_.filter_sigmas.initial_measurement_sigmas = {params_vec[0], params_vec[1], params_vec[2], params_vec[3]};
        this->get_parameter("stair_ekf_params.initial_pose_sigmas", params_vec);
        stair_manager_params_.filter_sigmas.initial_pose_sigmas = {params_vec[0], params_vec[1], params_vec[2], params_vec[3]};
        this->get_parameter("stair_ekf_params.measurement_sigmas", params_vec);
        stair_manager_params_.filter_sigmas.measurement_sigmas = {params_vec[0], params_vec[1], params_vec[2], params_vec[3]};
        this->get_parameter("stair_ekf_params.pose_sigmas", params_vec);
        stair_manager_params_.filter_sigmas.pose_sigmas = {params_vec[0], params_vec[1], params_vec[2], params_vec[3]};
        this->get_parameter("stair_ekf_params.model_noise_sigmas", params_vec);
        stair_manager_params_.filter_sigmas.staircase_model_sigmas << params_vec[0], params_vec[1], params_vec[2], params_vec[3], params_vec[4], params_vec[5];

        RCLCPP_INFO(this->get_logger(), "\033[1;35m [Staircase Robot Node] Using EKF for merging stairs! \033[0m");
        stair_manager_params_.filter_type = stair_utility::StaircaseFilterType::LocalFrameEKF;

    } else {
        RCLCPP_ERROR(this->get_logger(), "[Staircase Robot Node] Unknown filter type specified, using basic averaging!");
        stair_manager_params_.filter_type = stair_utility::StaircaseFilterType::SimpleAveraging;
    }

    // Initialize Stair Manager
    manager_ = SingleRobotStairManager(stair_manager_params_);

    // === 3. Create Publishers, Subscribers, and Timers ===
    // QoS profile for sensor data
    auto sensor_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort().durability_volatile();

    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(robots_topics_prefix_ + "/" + pointcloud_topic_, sensor_qos, std::bind(&StaircaseEstimationRobotNode::PointCloudHandler, this, _1));
    
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(robots_topics_prefix_ + "/" + odom_topic_, sensor_qos, std::bind(&StaircaseEstimationRobotNode::OdometryHandler, this, _1));
    
    global_tf_sub_ = this->create_subscription<geometry_msgs::msg::TransformStamped>(robots_topics_prefix_ + "/" + global_tf_topic_, 10, std::bind(&StaircaseEstimationRobotNode::GlobalTFHandler, this, _1));

    enable_staircase_sub_ = this->create_subscription<std_msgs::msg::Bool>(staircase_node_toggle_topic_, 10, std::bind(&StaircaseEstimationRobotNode::HandleEnable, this, _1));

    staircase_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(staircase_node_status_topic_, 10);
    
    stairs_pub_ = this->create_publisher<staircase_msgs::msg::StaircaseMsg>(staircase_estimates_topic_, 10);

    detection_timer_ = this->create_wall_timer(
        std::chrono::duration<double>(1.0 / ros_rate_),
        std::bind(&StaircaseEstimationRobotNode::PerceiveStaircases, this));

    if (debug_)
    {
        process_cl_pub1_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/staircase_estimation_robot_node/stair_debug_cloud1", 10);
        process_cl_pub2_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/staircase_estimation_robot_node/stair_debug_cloud2", 10);
        line_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/staircase_estimation_robot_node/stair_debug_line_marker", 10);
    }

    if (publish_staircase_measurement_)
    {
        measurement_pub_ = this->create_publisher<staircase_msgs::msg::StaircaseMeasurement>(staircase_measurements_topic_, 10);
    }

    RCLCPP_INFO(this->get_logger(), "\033[1;35m Staircase Node started, Debug: %d, Sim Time: %d \033[0m", debug_, simulation_);
}

void StaircaseEstimationRobotNode::PerceiveStaircases()
{   
    auto ts = std::chrono::high_resolution_clock::now();

    // Guard against an empty accumulated cloud: when the robot is in a region with no points
    // inside the crop box, lasercloud_stacked_ is empty (width == 0). pcl::transformPointCloud
    // then does an integer divide by the cloud width (PointCloud::assign -> height = size()/width),
    // which raises SIGFPE. Skip detection this cycle when there is nothing to process.
    if (new_lasercloud_ && enable_stair_detection_ && odom_initialized_ && !lasercloud_stacked_->empty())
    {
        auto t1 = std::chrono::high_resolution_clock::now();

        // In ROS 2, we use tf2::Transform for calculations.
        tf2::Transform transform_transl, transform_rot;
        transform_transl.setIdentity();
        transform_transl.setOrigin(tf2::Vector3(-vehicle_x_, -vehicle_y_, -vehicle_z_));
        
        transform_rot.setIdentity();
        tf2::Quaternion q;
        q.setRPY(0, 0, -vehicle_yaw_);
        transform_rot.setRotation(q);

        robot_pose_transform_ = transform_rot * transform_transl;

        // The pcl_ros::transformPointCloud function is still available
        pcl_ros::transformPointCloud(*lasercloud_stacked_, *lasercloud_local_, robot_pose_transform_);
        
        auto t2 = std::chrono::high_resolution_clock::now();

        detector_.setPointCloudAndOdometry(lasercloud_local_);
        stair_detected_ = detector_.detectStaircase(staircase_up_, staircase_down_);
        
        auto t3 = std::chrono::high_resolution_clock::now();
        
        odom_to_robot_tf_ = robot_pose_transform_.inverse();
        global_tf_ = global_to_odom_tf_ * odom_to_robot_tf_;

        if (transform_detections_to_global_) {
            robot_pos_.frame_id = global_frame_id_;
        } else {
            robot_pos_.frame_id = odom_frame_id_;
        }
        
        if (stair_detected_ != stair_utility::StaircaseDetectorResult::NoStairsDetected)
        {   
            tf2::Vector3 origin = odom_to_robot_tf_.getOrigin();
            tf2::Quaternion rotation = odom_to_robot_tf_.getRotation();

            robot_pos_.vehicle_pos = Eigen::Translation3d(origin.x(), origin.y(), origin.z());
            robot_pos_.vehicle_quat = Eigen::Quaterniond(rotation.w(), rotation.x(), rotation.y(), rotation.z());
            int st_id;

            bool is_confirmed = false;
            if(stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedUp || stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedBoth){
                staircase_up_.robot_pose = robot_pos_;
                st_id = manager_.addNewDetectedStaircase(staircase_up_, stair_up_estimate_, is_confirmed);

                if(publish_staircase_measurement_){
                    publishStaircaseMeasurement(staircase_up_, true);
                }
                // Only publish once the staircase is confirmed across enough world-frame observations.
                if(is_confirmed){
                    publishStaircaseEstimate(stair_up_estimate_);
                    last_published_estimates_[stair_up_estimate_.stair_id] = stair_up_estimate_;
                    last_detection_time_[stair_up_estimate_.stair_id] = this->now();
                }
            }
            if(stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedDown || stair_detected_ == stair_utility::StaircaseDetectorResult::StairsDetectedBoth){
                staircase_down_.robot_pose = robot_pos_;
                st_id = manager_.addNewDetectedStaircase(staircase_down_, stair_down_estimate_, is_confirmed);

                if(publish_staircase_measurement_){
                    publishStaircaseMeasurement(staircase_down_, false);
                }
                // Only publish once the staircase is confirmed across enough world-frame observations.
                if(is_confirmed){
                    publishStaircaseEstimate(stair_down_estimate_);
                    last_published_estimates_[stair_down_estimate_.stair_id] = stair_down_estimate_;
                    last_detection_time_[stair_down_estimate_.stair_id] = this->now();
                }
            }

            auto t4 = std::chrono::high_resolution_clock::now();
            auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1);
            auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2);
            auto d3 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3);
            
            // RCLCPP_INFO(this->get_logger(), "\033[1;35m[Staircase Robot Node Times] PointCloud: %ld, Detection: %ld, Matching and Merging: %ld \033[0m", d1.count(), d2.count(), d3.count());
        }
        else
        {
            // Detector found nothing this frame: keep nearby staircases alive so a momentary
            // dropout doesn't make a clearly-present staircase disappear.
            republishRecentStaircases();
        }

        // new_lasercloud_ = false;

        if (debug_)
        {   
            // Publish input voxel grid - Transform to global if necessary
            pcl_ros::transformPointCloud(*lasercloud_stacked_, *lasercloud_processed2_, global_to_odom_tf_);
            
            sensor_msgs::msg::PointCloud2 debug_cloud1;
            pcl::toROSMsg(*lasercloud_processed2_, debug_cloud1);
            debug_cloud1.header.stamp = this->now();

            if (transform_detections_to_global_) {
                debug_cloud1.header.frame_id = global_frame_id_;
            } else {
                debug_cloud1.header.frame_id = odom_frame_id_;
            }
            process_cl_pub1_->publish(debug_cloud1);

            /// Publish processed cloud. Stage - 1 for top-view, 2 for cylindrical-view
            detector_.getProcessedCloud(lasercloud_processed_, 2);
            pcl_ros::transformPointCloud(*lasercloud_processed_, *lasercloud_processed2_, global_tf_);
            
            sensor_msgs::msg::PointCloud2 debug_cloud2;
            pcl::toROSMsg(*lasercloud_processed2_, debug_cloud2);
            debug_cloud2.header.stamp = this->now();

            // Set the frame_id correctly based on the transform used.
            if (transform_detections_to_global_) {
                debug_cloud2.header.frame_id = global_frame_id_;
            } else {
                debug_cloud2.header.frame_id = odom_frame_id_;
            }
            process_cl_pub2_->publish(debug_cloud2);

            publishLineMarkers();
        }
    }

    if ((this->now() - last_heartbeat_publish_time_).seconds() > 2.0)
    {
        staircase_heartbeat_msg_.data = enable_stair_detection_;
        staircase_status_pub_->publish(staircase_heartbeat_msg_);
        last_heartbeat_publish_time_ = this->now();
    }

    if(!enable_stair_detection_){
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 100, "Staircase detection has been disabled!");
    }

    // auto te = std::chrono::high_resolution_clock::now();
    // auto d = std::chrono::duration_cast<std::chrono::microseconds>(te - ts);
    // RCLCPP_INFO(this->get_logger(), "[Staircase Robot Node Loop Time] PointCloud: %ld", d.count());
}


void StaircaseEstimationRobotNode::PointCloudHandler(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    if (enable_stair_detection_ && odom_initialized_)
    {
        lasercloud_->clear();
        pcl::fromROSMsg(*msg, *lasercloud_);

        for(size_t i = 0; i < lasercloud_->points.size(); i++){
            lasercloud_stacked_->points.push_back(lasercloud_->points[i]);
        }
        
        box_filter_.setMin(Eigen::Vector4f(min_x_, min_y_, min_z_, -1.0));
        box_filter_.setMax(Eigen::Vector4f(max_x_, max_y_, max_z_, 1.0));
        box_filter_.setInputCloud(lasercloud_stacked_);
        box_filter_.filter(*lasercloud_cropped_);

        voxel_grid_filter_.setInputCloud(lasercloud_cropped_);
        voxel_grid_filter_.setLeafSize(leaf_size_, leaf_size_, leaf_size_);  
        voxel_grid_filter_.filter(*lasercloud_stacked_);
        
        new_lasercloud_ = true;
    }
}

void StaircaseEstimationRobotNode::OdometryHandler(const nav_msgs::msg::Odometry::SharedPtr msg)
{   
    tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(vehicle_roll_, vehicle_pitch_, vehicle_yaw_);

    vehicle_x_ = msg->pose.pose.position.x;
    vehicle_y_ = msg->pose.pose.position.y;
    vehicle_z_ = msg->pose.pose.position.z;

    vehicle_pose_ = msg->pose.pose;

    max_x_ = max_range_x_ + vehicle_x_;
    min_x_ = min_range_x_ + vehicle_x_;
    max_y_ = max_range_y_ + vehicle_y_;
    min_y_ = min_range_y_ + vehicle_y_;
    max_z_ = max_range_z_ + vehicle_z_;
    min_z_ = min_range_z_ + vehicle_z_;

    odom_initialized_ = true;
}

void StaircaseEstimationRobotNode::GlobalTFHandler(const geometry_msgs::msg::TransformStamped::SharedPtr msg)
{
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 15000, "[Staircase Robot Node] Received and Updated Global TF");
    global_to_odom_tf_stamped_.transform = msg->transform;
    global_to_odom_tf_stamped_.header = msg->header;

    tf2::fromMsg(global_to_odom_tf_stamped_.transform, global_to_odom_tf_);
}

void StaircaseEstimationRobotNode::HandleEnable(const std_msgs::msg::Bool::SharedPtr msg)
{
    if (msg->data)
    {
        enable_stair_detection_ = true;
        RCLCPP_WARN(this->get_logger(), "ENABLED STAIRCASE DETECTION");
    }
    else
    {
        enable_stair_detection_ = false;
        RCLCPP_WARN(this->get_logger(), "DISABLED STAIRCASE DETECTION");
    }
}

// Placeholder implementations for the publish functions
void StaircaseEstimationRobotNode::publishStaircaseMeasurement(const stair_utility::StaircaseMeasurement& measurement, bool ascending) {
    auto msg = std::make_unique<staircase_msgs::msg::StaircaseMeasurement>();
    
    msg->stair_count = measurement.stair_count;
    msg->frame_id = measurement.robot_pose.frame_id;
    msg->robot_transform = tf2::toMsg(global_tf_);
    
    for(const auto& step : measurement.steps) {
        geometry_msgs::msg::Point start, end;
        start.x = step.start_p(0);
        start.y = step.start_p(1);
        start.z = step.start_p(2);
        end.x = step.end_p(0);
        end.y = step.end_p(1);
        end.z = step.end_p(2);
        msg->steps_start_p.push_back(start);
        msg->steps_end_p.push_back(end);
        msg->step_lengths.push_back(step.step_width);
        msg->step_radii.push_back(step.line_polar_form(0));
        msg->step_angles.push_back(step.line_polar_form(1));
        msg->r_r_covariance.push_back(step.step_covariance(0, 0));
        msg->r_t_covariance.push_back(step.step_covariance(1, 0));
        msg->t_t_covariance.push_back(step.step_covariance(1, 1));
        msg->z_covariance.push_back(step.step_covariance(2, 2));
    }
    msg->is_ascending = ascending;
    
    measurement_pub_->publish(std::move(msg));
}

void StaircaseEstimationRobotNode::publishStaircaseEstimate(const stair_utility::StaircaseEstimate& estimate) {
    auto stair_msg = std::make_unique<staircase_msgs::msg::StaircaseMsg>();

    stair_msg->stair_id = estimate.stair_id;
    stair_msg->stair_count = estimate.stair_count;
    stair_msg->robot_id = robot_name_;

    if (transform_detections_to_global_){
        stair_msg->frame_id = global_frame_id_;
    } else {
        stair_msg->frame_id = odom_frame_id_;
    }

    tf2::Transform global_to_odom_tf;
    tf2::fromMsg(global_to_odom_tf_stamped_.transform, global_to_odom_tf);

    for(const auto& step : estimate.steps) {
        geometry_msgs::msg::Point start, end;
        tf2::Vector3 point_s(step.start_p(0), step.start_p(1), step.start_p(2));
        tf2::Vector3 point_e(step.end_p(0), step.end_p(1), step.end_p(2));
        
        tf2::Vector3 point_s_transformed = global_to_odom_tf * point_s;
        start.x = point_s_transformed.x();
        start.y = point_s_transformed.y();
        start.z = point_s_transformed.z();
        
        tf2::Vector3 point_e_transformed = global_to_odom_tf * point_e;
        end.x = point_e_transformed.x();
        end.y = point_e_transformed.y();
        end.z = point_e_transformed.z();

        stair_msg->steps_start_p.push_back(start);
        stair_msg->steps_end_p.push_back(end);

        stair_msg->steps_radii.push_back(step.line_polar_form(0));
        stair_msg->steps_theta.push_back(step.line_polar_form(1));

        stair_msg->steps_radii_covariances.push_back(step.step_covariance(0, 0));
        stair_msg->steps_theta_covariances.push_back(step.step_covariance(1, 1));
        stair_msg->start_z_covariances.push_back(step.step_covariance(2, 2));
        stair_msg->end_z_covariances.push_back(step.step_covariance(3, 3));

    }

    stair_msg->stair_height = estimate.staircase_parameters[0];
    stair_msg->stair_depth = estimate.staircase_parameters[1];
    stair_msg->stair_curvature = estimate.staircase_parameters[4];
    stair_msg->stair_width = estimate.staircase_parameters[2];

    stairs_pub_->publish(std::move(stair_msg));
}

// Re-publish the most recently confirmed staircases that are still within
// detection_retention_range_ of the robot. Called on frames where the detector returned
// nothing, so a momentary dropout doesn't make a nearby, clearly-present staircase vanish.
// Distance is the nearest step to the robot in 3D (odom frame). The vertical (z) term is
// essential in multi-floor buildings: a staircase one floor below sits at nearly the same
// (x, y) but several meters down, so a 2D check would wrongly keep re-publishing it.
void StaircaseEstimationRobotNode::republishRecentStaircases() {
    if (detection_retention_range_ <= 0.0 || last_published_estimates_.empty())
        return;

    const double range_sq = detection_retention_range_ * detection_retention_range_;

    for (const auto& entry : last_published_estimates_) {
        const int stair_id = entry.first;
        const stair_utility::StaircaseEstimate& estimate = entry.second;
        if (estimate.steps.empty())
            continue;

        // Time gate: only bridge brief dropouts. If the detector hasn't actually seen this
        // staircase for longer than detection_hold_time_, stop re-publishing it (the robot can
        // no longer see it), so it isn't shown indefinitely just for being nearby.
        if (detection_hold_time_ > 0.0) {
            auto t_it = last_detection_time_.find(stair_id);
            if (t_it == last_detection_time_.end() ||
                (this->now() - t_it->second).seconds() > detection_hold_time_) {
                continue;
            }
        }

        double min_dist_sq = std::numeric_limits<double>::max();
        for (const auto& step : estimate.steps) {
            const double dxs = step.start_p(0) - vehicle_x_;
            const double dys = step.start_p(1) - vehicle_y_;
            const double dzs = step.start_p(2) - vehicle_z_;
            min_dist_sq = std::min(min_dist_sq, dxs * dxs + dys * dys + dzs * dzs);
            const double dxe = step.end_p(0) - vehicle_x_;
            const double dye = step.end_p(1) - vehicle_y_;
            const double dze = step.end_p(2) - vehicle_z_;
            min_dist_sq = std::min(min_dist_sq, dxe * dxe + dye * dye + dze * dze);
        }

        if (min_dist_sq <= range_sq) {
            publishStaircaseEstimate(estimate);
        }
    }
}

void StaircaseEstimationRobotNode::publishLineMarkers() {
    auto marker_array_msg = std::make_unique<visualization_msgs::msg::MarkerArray>();
    
    visualization_msgs::msg::Marker marker_up, marker_down, marker_ground;
    
    // Configure 'up' marker
    if (transform_detections_to_global_){
        marker_up.header.frame_id = global_frame_id_;
    } else {
        marker_up.header.frame_id = odom_frame_id_;
    }

    marker_up.header.stamp = this->now();
    marker_up.ns = "line_extraction_marker_up";
    marker_up.id = 1;
    marker_up.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker_up.action = visualization_msgs::msg::Marker::ADD;
    marker_up.scale.x = 0.05;
    marker_up.color.b = 1.0;
    marker_up.color.a = 1.0;
    // Expire if not refreshed, so debug lines don't freeze in the world frame when the robot
    // moves on / the detector stops producing lines (they are re-published every cycle while active).
    marker_up.lifetime = rclcpp::Duration::from_seconds(debug_marker_lifetime_);

    // Configure 'down' marker
    marker_down = marker_up; // Copy properties (incl. lifetime)
    marker_down.ns = "line_extraction_marker_down";
    marker_down.id = 2;
    marker_down.color.r = 1.0;
    marker_down.color.g = 0.0;
    marker_down.color.b = 0.0;

    // Configure 'ground' marker
    marker_ground = marker_up; // Copy properties
    marker_ground.ns = "line_extraction_marker_ground";
    marker_ground.id = 3; // Different ID
    marker_ground.color.r = 0.0;
    marker_ground.color.g = 1.0;
    marker_ground.color.b = 0.0;

    geometry_msgs::msg::TransformStamped transform_stamped;
    transform_stamped.transform = tf2::toMsg(global_tf_);

    // Get all lines above ground
    const auto& linesAbove = detector_.getDetectedLinesAbove();
    for (const auto& lines : linesAbove) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];

            geometry_msgs::msg::Point p_start_tf, p_end_tf;
            tf2::doTransform(p_start, p_start_tf, transform_stamped);
            tf2::doTransform(p_end, p_end_tf, transform_stamped);
            
            marker_up.points.push_back(p_start_tf);
            marker_up.points.push_back(p_end_tf);
        }
    }
    marker_array_msg->markers.push_back(marker_up);

    // Get all lines below ground
    const auto& linesBelow = detector_.getDetectedLinesBelow();
    for (const auto& lines : linesBelow) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];

            geometry_msgs::msg::Point p_start_tf, p_end_tf;
            tf2::doTransform(p_start, p_start_tf, transform_stamped);
            tf2::doTransform(p_end, p_end_tf, transform_stamped);

            marker_down.points.push_back(p_start_tf);
            marker_down.points.push_back(p_end_tf);
        }
    }
    marker_array_msg->markers.push_back(marker_down);

    // Get all lines on ground
    const auto& linesGround = detector_.getDetectedLinesGround();
    for (const auto& lines : linesGround) {
        for(size_t i = 0; i < lines->size(); i++){
            geometry_msgs::msg::Point p_start, p_end;
            p_start.x = lines->at(i).line_start[0];
            p_start.y = lines->at(i).line_start[1];
            p_start.z = lines->at(i).line_start[2];
            p_end.x = lines->at(i).line_end[0];
            p_end.y = lines->at(i).line_end[1];
            p_end.z = lines->at(i).line_end[2];
            
            geometry_msgs::msg::Point p_start_tf, p_end_tf;
            tf2::doTransform(p_start, p_start_tf, transform_stamped);
            tf2::doTransform(p_end, p_end_tf, transform_stamped);
            
            marker_ground.points.push_back(p_start_tf);
            marker_ground.points.push_back(p_end_tf);
        }
    }
    marker_array_msg->markers.push_back(marker_ground);

    line_marker_pub_->publish(std::move(marker_array_msg));
}

// Main function to run the node
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<StaircaseEstimationRobotNode>();
    RCLCPP_INFO(node->get_logger(), "\033[1;35m Starting Staircase ROS 2 Node for Robot \033[0m");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
