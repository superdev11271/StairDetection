#ifndef _STAIR_UTILITY_H_
#define _STAIR_UTILITY_H_

#include <vector>
#include <cmath>
#include <pcl/point_types.h>
#include <Eigen/Dense>
#include <chrono>

const double PI = 3.1415;

namespace stair_utility
{   
    struct PrecomputedCache{
        std::vector<unsigned int> indices;
        std::vector<float> bearings;
        std::vector<float> sin_bearings;
        std::vector<float> cos_bearings;
    };

    struct RangeData{
        std::vector<float> ranges;
        std::vector<float> xs;
        std::vector<float> ys;
        std::vector<float> zs;
    };

    struct RobotPositionInfo{
        Eigen::Quaterniond vehicle_quat; // Ordering is w, x, y, z;
        Eigen::Translation3d vehicle_pos;
        std::string frame_id;
    };

    struct StaircaseProcessingResult{
        
        bool success;

        int misc_time;
        int data_association_time;
        int model_prediction_time;
        int filter_time;
    };

    struct StaircaseDetectorParams{
        
        bool use_ramp_detection;

        double angle_resolution;
        double leaf_size;

        float robot_height;
        float initialization_range;
        float ground_height_buffer;

        int min_stair_count;
        double stair_slope_min;
        double stair_slope_max;
        
        double min_stair_width;
        double min_stair_height;
        double max_stair_height;
        double min_stair_depth;
        double max_stair_depth;
        double max_stair_curvature;

        // Detection gating (all disabled when <= 0.0 so callers that don't set them are unaffected)
        double max_detection_range = -1.0;        // [m] reject lines/steps whose center is farther than this from the robot
        double max_line_fit_stddev = -1.0;        // [m] reject lines whose radius-fit std-dev exceeds this (line fit quality)
        double max_step_depth_variation = -1.0;   // [m] reject staircases whose inter-step depth std-dev exceeds this (regularity)
        double max_step_height_variation = -1.0;  // [m] reject staircases whose inter-step height std-dev exceeds this (regularity)

        float x_max;
        float x_min;
        float y_max;
        float y_min;
        float z_max;
        float z_min;
    };

    enum StaircaseDetectorResult{
        NoStairsDetected = 0,
        StairsDetectedUp = 1,
        StairsDetectedDown = 2,
        StairsDetectedBoth = 3
    };
    
    enum StaircaseFilterType{
        SimpleAveraging = 1,
        LocalFrameEKF = 2,
        SimpleMaximum = 3
    };

    struct constantWidthEKFParams{
        // Initial Sigmas
        Eigen::Vector4d initial_pose_sigmas;
        Eigen::Vector4d initial_measurement_sigmas;

        // Noise for new measurement
        Eigen::Vector4d pose_sigmas;
        Eigen::Vector4d measurement_sigmas;

        // Noise for Model Prediction
        Eigen::Vector4d process_noise_sigmas;
        Eigen::Matrix<double, 6, 1> staircase_model_sigmas;
        
    };

    struct StairManagerParams{
        std::string robot_name; // [robot name]
        float yaw_threshold; // [rad]

        // Number of times a staircase must be (re)observed and associated in the world frame
        // before it is published. 1 publishes on first detection (gate disabled); higher values
        // suppress pose-dependent false positives that only appear from a single viewpoint.
        int min_detections_to_confirm = 1;

        StaircaseFilterType filter_type;
        constantWidthEKFParams filter_sigmas;

    };
    
    struct DetectedLine{
        std::array<double, 3> line_start;
        std::array<double, 3> line_center;
        std::array<double, 3> line_end;

        double line_radius, line_theta, line_z_variance;
        std::array<double, 4> line_covariance;

        double line_length, line_yaw_angle;
        bool skip;

        DetectedLine(std::array<double, 3> start, std::array<double, 3> end, 
         std::array<double, 3> center, double yaw_angle, double line_length, double radius, double theta, std::array<double, 4> covariance, double z_var)
        : line_start(std::move(start)), 
          line_end(std::move(end)),
          line_center(std::move(center)),
          line_yaw_angle(yaw_angle),
          line_length(line_length),
          line_radius(radius),
          line_theta(theta),
          line_covariance(std::move(covariance)),
          line_z_variance(z_var){skip = false;}

        DetectedLine(){}
    };

    struct StairStep{
        Eigen::Vector3d start_p, end_p;
        Eigen::Vector2d line_polar_form;
        Eigen::Matrix4d step_covariance;

        float step_width;
    };

    struct StaircaseMeasurement{
        uint8_t stair_count;
        std::vector<StairStep> steps;

        RobotPositionInfo robot_pose;
    };

    struct StaircaseEstimate{
        int stair_id;
        
        uint8_t stair_count;
        std::vector<StairStep> steps;
        Eigen::Matrix<double, 6, 1> staircase_parameters;
    };

    struct StaircaseInfo{
        std::vector<Eigen::Vector3d> stair_polygon; // Polygon to check if two staircases are the same
        float staircase_direction; // Direction to ensure two staircases are similar
    };

    struct SingleStaircaseSummary{
        int id;
        float stair_width, stair_depth, stair_height;
        float stair_start_direction, stair_end_direction;
        std::vector<std::string> robot_list; 
    };

    struct LineExtractorParams{
        double bearing_var;
        double range_var;
        double z_var;
        double least_sq_angle_thresh;
        double least_sq_radius_thresh;
        double max_line_gap;
        double min_line_length;
        double min_range;
        double max_range;
        double min_split_dist;
        double outlier_dist;
        unsigned int min_line_points;
    };

    struct PointParams
    {
        std::vector<double> a;
        std::vector<double> ap;
        std::vector<double> app;
        std::vector<double> b;
        std::vector<double> bp;
        std::vector<double> bpp;
        std::vector<double> c;
        std::vector<double> s;
    };

    inline float wrap2PI(float angle){
        angle = fmod(angle, 2 * M_PI);
        if (angle >= M_PI)
            angle -= 2 * M_PI;
        if(angle < -M_PI)
            angle += 2 * M_PI;
        return angle;
    }   

    inline float wrap2PIDegrees(float angle){
        float an = fmod(angle, 360);
        if(an >= 180)
            an -= 360;

        return an;
    }

    /* Returns Absolute angle difference between two lines */
    inline float get_line_angle_diff(float angle1, float angle2){
        float angle = fabs(angle1 - angle2);
        if(angle <= M_PI_2)
            return angle;
        else    
            return (M_PI - angle);

    }

    inline float get_xy_distance(const pcl::PointXYZI &point1, const pcl::PointXYZI &point2){
        
        float dist = sqrt((point1.x - point2.x)*(point1.x - point2.x) + (point1.y - point2.y)*(point1.y - point2.y));
        return dist;
    }

    inline float get_xy_distance(const std::array<double,3> &point1, const std::array<double,3> &point2){
        
        float dist = sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
        return dist;
    }

    inline float get_xy_distance(const std::array<float,3> &point1, const std::array<float,3> &point2){
        
        float dist = sqrt((point1[0] - point2[0])*(point1[0] - point2[0]) + (point1[1] - point2[1])*(point1[1] - point2[1]));
        return dist;
    }

    inline float get_line_distance(const std::array<double,3> &p1, const std::array<double,3> &p2, const std::array<double,3> &p3){
        float A1 = p2[1] - p1[1];
        float B1 = p1[0] - p2[0];
        float C1 = (p1[1]*p2[0] - p1[0]*p2[1]);
        
        return  fabs(A1*p3[0] + B1*p3[1] + C1)/sqrt(A1*A1 + B1*B1);
    }

    inline float get_line_distance(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, const Eigen::Vector3d &p3){
        float A1 = p2(1) - p1(1);
        float B1 = p1(0) - p2(0);
        float C1 = (p1(1)*p2(0) - p1(0)*p2(1));
        
        return  fabs(A1*p3(0) + B1*p3(1) + C1)/sqrt(A1*A1 + B1*B1);
    }
    
    inline float get_line_distance(const Eigen::Vector2d &p1, const Eigen::Vector2d &p2, const Eigen::Vector2d &p3){
        float A1 = p2(1) - p1(1);
        float B1 = p1(0) - p2(0);
        float C1 = (p1(1)*p2(0) - p1(0)*p2(1));
        
        return  fabs(A1*p3(0) + B1*p3(1) + C1)/sqrt(A1*A1 + B1*B1);
    }
    
    inline void computeAveragePoint(const std::array<float,3> &p1, const std::array<float,3> &p2, std::array<float,3> &p3 ){
        p3[0] = (p1[0] + p2[0])/2;
        p3[1] = (p1[1] + p2[1])/2;
        p3[2] = (p1[2] + p2[2])/2;
    }
    
    inline bool crossProduct2d(const Eigen::Vector3d &vec1, const Eigen::Vector3d &vec2){
        return (vec1(0)*vec2(1)) - (vec1(1)*vec2(0));
    }

    template <typename PointT, typename PolygonPointT> // Works for Eigen::Vector2d, Eigen::Vector3d and pcl::PointXYZ
    static bool isPointInPolygon(const PointT& point, const std::vector<PolygonPointT>& polygon) {
        int num_vertices = polygon.size();
        bool inside = false;

        // Store the first point in the polygon and initialize the second point
        PolygonPointT p1 = polygon[0], p2;

        // Loop through each edge in the polygon
        for (int i = 1; i <= num_vertices; i++) {
            // Get the next point in the polygon
            p2 = polygon[i % num_vertices];

            // Check if the point is above the minimum y coordinate of the edge
            if (point.y() > std::min(p1.y(), p2.y())) {
                // Check if the point is below the maximum y coordinate of the edge
                if (point.y() <= std::max(p1.y(), p2.y())) {
                    // Check if the point is to the left of the maximum x coordinate of the edge
                    if (point.x() <= std::max(p1.x(), p2.x())) {
                        // Calculate the x-intersection of the line connecting the point to the edge
                        double x_intersection = (point.y() - p1.y()) * (p2.x() - p1.x()) / (p2.y() - p1.y()) + p1.x();

                        // Check if the point is on the same line as the edge or to the left of the x-intersection
                        if (p1.x() == p2.x() || point.x() <= x_intersection) {
                            // Flip the inside flag
                            inside = !inside;
                        }
                    }
                }
            }

            // Store the current point as the first point for the next iteration
            p1 = p2;
        }

        // Return the value of the inside flag
        return inside;
    }
} 


// namespace {stair_utility}

#endif