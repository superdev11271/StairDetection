#ifndef _STAIR_DETECTOR_H_
#define _STAIR_DETECTOR_H_

#include <cmath>
#include <random>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/io/pcd_io.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/Vertices.h>

#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/utils/line_extraction/line_extractor.hpp"
#include "staircase_perception/utils/line_extraction/line.hpp"

#include <string>
#include <iostream>
#include <algorithm>
#include <vector>
#include <stdlib.h>

class StairDetector
{
    public:
        
        StairDetector();
        StairDetector(stair_utility::StaircaseDetectorParams detector_params, stair_utility::LineExtractorParams line_params);
        
        ~StairDetector();

        void setPointCloudAndOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud);
        void setPointCloudAndOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, float r_height);

        stair_utility::StaircaseDetectorResult detectStaircase(stair_utility::StaircaseMeasurement& stair_up, stair_utility::StaircaseMeasurement& stair_down);
        void getProcessedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr processed, int stage);

        // Getter for detected_lines_above_
        const std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>>& getDetectedLinesAbove() const {
            return detected_lines_above_;
        }

        // Getter for detected_lines_below_
        const std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>>& getDetectedLinesBelow() const {
            return detected_lines_below_;
        }

        // Getter for detected_lines_ground_
        const std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>>& getDetectedLinesGround() const {
            return detected_lines_ground_;
        }

    private:

        // Functions
        void SegmentPointCloud();
        void getLinesFromCloud();
        // Removes extracted lines that are too far from the robot or whose fit is unreliable.
        void filterLowQualityLines(const std::shared_ptr<std::deque<stair_utility::DetectedLine>>& lines);
        // Returns false if the assembled steps are too irregular in depth/height to be a real staircase.
        bool isStaircaseConsistent(const std::vector<stair_utility::DetectedLine>& steps) const;
        bool searchForAscendingStairs();
        bool searchForDescendingStairs();
        
        void populateAscendingStairs(stair_utility::StaircaseMeasurement& stair_up);
        void populateDescendingStairs(stair_utility::StaircaseMeasurement& stair_down);

        bool mark_detection_;

        // Point Cloud Parameters - For Pre-processing and filtering
        double leaf_size_;
        float angle_resolution_, sector_size_;
        double y_min_, y_max_, z_min_, x_max_, z_max_, x_min_;
        bool use_ramp_detection_;

        int cloud_ang_width_, cloud_z_height_;
        int cloud_width_, cloud_length_;
        float robot_height_;
        int ground_index_;
        float initialization_distance_, ground_height_buffer_;

        //Line Extractor Parameter Varaibles
        double bearing_variance_, range_variance_, z_variance_, least_sq_angle_thresh_, least_sq_radius_thresh_;
        double max_line_gap_, min_line_length_, min_range_, max_range_, min_split_distance_, outlier_distance_;
        int min_line_points_;

        int min_stair_count_;
        double stair_slope_min_, stair_slope_max_;
        double min_stair_width_, min_stair_height_, max_stair_height_;
        double min_stair_depth_, max_stair_depth_, max_stair_curvature_;

        // Detection gating thresholds (<= 0.0 disables the corresponding gate)
        double max_detection_range_, max_line_fit_stddev_;
        double max_step_depth_variation_, max_step_height_variation_;

        bool stair_detected_;
        int stair_initialization_range_, ground_line_padding_size_;

        pcl::PointCloud<pcl::PointXYZI>::Ptr voxelgrid_input_;
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_projected_; //Top Down projected Cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr lasercloud_cylindrical_; //Cylindrical Projected Cloud

        //Line Extractor Object;
        LineExtractor line_extractor_;
        // std::vector<std::vector<Line>> detected_lines_above_, detected_lines_below_, detected_lines_ground_;
        std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>> detected_lines_above_;
        std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>> detected_lines_ground_;
        std::vector<std::shared_ptr<std::deque<stair_utility::DetectedLine>>> detected_lines_below_;

        //Detected Stair Lines
        std::vector<stair_utility::DetectedLine> stairs_up_, stairs_down_;
        std::vector<stair_utility::DetectedLine> ramp_lines_up_, ramp_lines_down_;

};

#endif
