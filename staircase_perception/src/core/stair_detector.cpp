#include "staircase_perception/core/stair_detector.hpp"

//Default Constructors and Destructors
StairDetector::StairDetector(){}
StairDetector::~StairDetector(){}

//Constructor with parameter initialzers
StairDetector::StairDetector(stair_utility::StaircaseDetectorParams detector_params, stair_utility::LineExtractorParams line_params)
{   
    //Store the Detector Params in the variables
    angle_resolution_ = detector_params.angle_resolution;
    leaf_size_ = detector_params.leaf_size;
    x_max_ = detector_params.x_max;
    x_min_ = detector_params.x_min;
    y_max_ = detector_params.y_max;
    y_min_ = detector_params.y_min;
    z_max_ = detector_params.z_max;
    z_min_ = detector_params.z_min;

    robot_height_ = detector_params.robot_height;
    min_stair_count_ = detector_params.min_stair_count;
    stair_slope_min_ = detector_params.stair_slope_min;
    stair_slope_max_ = detector_params.stair_slope_max;

    min_stair_width_ = detector_params.min_stair_width;
    
    min_stair_height_ = detector_params.min_stair_height;
    max_stair_height_ = detector_params.max_stair_height;
    
    min_stair_depth_ = detector_params.min_stair_depth;
    max_stair_depth_ = detector_params.max_stair_depth;

    max_stair_curvature_ = detector_params.max_stair_curvature;

    max_detection_range_ = detector_params.max_detection_range;
    max_line_fit_stddev_ = detector_params.max_line_fit_stddev;
    max_step_depth_variation_ = detector_params.max_step_depth_variation;
    max_step_height_variation_ = detector_params.max_step_height_variation;

    initialization_distance_ = detector_params.initialization_range;
    ground_height_buffer_ = detector_params.ground_height_buffer;
    use_ramp_detection_ = detector_params.use_ramp_detection;

    voxelgrid_input_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_projected_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
    lasercloud_cylindrical_ = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());

    cloud_ang_width_ = 360.0 / angle_resolution_;
    cloud_z_height_ = (z_max_ - z_min_)/leaf_size_;

    cloud_length_ = (x_max_ - x_min_)/leaf_size_;
    cloud_width_ = (y_max_ - y_min_)/leaf_size_;

    std::cout << "\033[1;32m[Staircase Detector]Segmentation Parameters: \033[0m" << std::endl;
    std::cout << "\033[1;32mTop Down Projection -- Length: " << cloud_length_ <<", Width: " << cloud_width_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32mCylindrical Projection -- Width: "<< cloud_ang_width_ <<", Height:" << cloud_z_height_ << "\033[0m" << std::endl;

    lasercloud_projected_->width = cloud_width_;
    lasercloud_projected_->height = cloud_length_;
    lasercloud_projected_->is_dense = false;
    lasercloud_projected_->points.resize(cloud_width_ * cloud_length_);

    lasercloud_cylindrical_->width = cloud_ang_width_;
    lasercloud_cylindrical_->height = cloud_z_height_;
    lasercloud_cylindrical_->is_dense = false;
    lasercloud_cylindrical_->points.resize(cloud_ang_width_ * cloud_z_height_);

    stair_detected_ = false;

    //Setting Line Extractor Parameters fot the Object
    bearing_variance_ = line_params.bearing_var;
    range_variance_ = line_params.range_var;
    z_variance_ = line_params.z_var;
    least_sq_angle_thresh_ = line_params.least_sq_angle_thresh; 
    least_sq_radius_thresh_ = line_params.least_sq_radius_thresh;
    max_line_gap_ = line_params.max_line_gap;
    min_range_ = line_params.min_range;
    max_range_ = line_params.max_range;
    min_split_distance_ = line_params.min_split_dist;
    min_line_points_ = line_params.min_line_points;
    outlier_distance_ = line_params.outlier_dist;

    min_line_length_ = min_stair_width_; // Minimum line width for segmentation

    line_extractor_.setBearingVariance(bearing_variance_);
    line_extractor_.setRangeVariance(range_variance_);
    line_extractor_.setZVariance(z_variance_);
    line_extractor_.setLeastSqAngleThresh(least_sq_angle_thresh_);
    line_extractor_.setLeastSqRadiusThresh(least_sq_radius_thresh_);
    line_extractor_.setMaxLineGap(max_line_gap_);
    line_extractor_.setMinLineLength(min_line_length_);
    line_extractor_.setMinRange(min_range_);
    line_extractor_.setMaxRange(max_range_);
    line_extractor_.setMinSplitDist(min_split_distance_);
    line_extractor_.setMinLinePoints(min_line_points_);
    line_extractor_.setOutlierDist(outlier_distance_);

    //Set Line Extractor Precomputed Cache
    std::vector<float> bearings, cos_bearings, sin_bearings;
    std::vector<unsigned int> indices;
    
    for(int indi = 0; indi < cloud_ang_width_; indi++ ){
        float b = -M_PI + (indi*(M_PI/180)); 
        bearings.push_back(b);
        cos_bearings.push_back(cos(b));
        sin_bearings.push_back(sin(b));
        indices.push_back(indi);
    }

    line_extractor_.setPrecomputedCache(bearings, cos_bearings, sin_bearings, indices);
    ground_index_ = round((- robot_height_ - z_min_) * cloud_z_height_ /(z_max_ - z_min_));
    stair_initialization_range_ = round(initialization_distance_ / leaf_size_);
    ground_line_padding_size_ = round(ground_height_buffer_ / leaf_size_);

    std::cout << "\033[1;32mRobot height: " << robot_height_ << " Ground Index: " << ground_index_ << "\033[0m" << std::endl;
    std::cout << "\033[1;32mInitialization Range index: " << stair_initialization_range_<< " Ground Padding range: " << ground_line_padding_size_ << "\033[0m" << std::endl;
    detected_lines_below_.reserve(ground_index_ - ground_line_padding_size_ + 1);
    detected_lines_ground_.reserve(2 * ground_line_padding_size_ + 1);
    detected_lines_above_.reserve(cloud_z_height_ - ground_index_ - ground_line_padding_size_);

    for (int i = 0; i < cloud_z_height_; ++i) {
        if(i < ground_index_ - ground_line_padding_size_)
            detected_lines_below_.push_back(std::make_shared<std::deque<stair_utility::DetectedLine>>());
        else if(i >= ground_index_ - ground_line_padding_size_ && i <= ground_index_ + ground_line_padding_size_)
            detected_lines_ground_.push_back(std::make_shared<std::deque<stair_utility::DetectedLine>>());
        else
            detected_lines_above_.push_back(std::make_shared<std::deque<stair_utility::DetectedLine>>());
    }

} // end Constructor

// Set point cloud to detect staircase. Point cloud needs to be in local frame (centered around the robot's position)
void StairDetector::setPointCloudAndOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud){
    voxelgrid_input_ = input_cloud;
}

// Setting staircase detection inputs in case of variable robot height (Spot with feedback of its height)
// Default height gets overriden when this is used, use this to reset a height if necessary
void StairDetector::setPointCloudAndOdometry(const pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud, float r_height){
    voxelgrid_input_ = input_cloud;

    robot_height_ = r_height;
    ground_index_ = round((- robot_height_ - z_min_) * cloud_z_height_ /(z_max_ - z_min_));
    std::cout << "\033[1;32mRobot height: " << robot_height_ << " Ground Index: " << ground_index_ << "\033[0m" << std::endl;
}

stair_utility::StaircaseDetectorResult StairDetector::detectStaircase(stair_utility::StaircaseMeasurement& stair_up, stair_utility::StaircaseMeasurement& stair_down){
    mark_detection_ = false;
    for(int k = 0; k < detected_lines_above_.size(); k++) {
        detected_lines_above_[k]->clear();
    }
    for(int k = 0; k < detected_lines_below_.size(); k++) {
        detected_lines_below_[k]->clear();
    }
    for(int k = 0; k < detected_lines_ground_.size(); k++) {
        detected_lines_ground_[k]->clear();
    }

    stair_utility::StaircaseDetectorResult result = stair_utility::StaircaseDetectorResult::NoStairsDetected;

    this->SegmentPointCloud();
    this->getLinesFromCloud();
    mark_detection_ = this->searchForAscendingStairs();
    if(mark_detection_){
            this->populateAscendingStairs(stair_up);
            result = stair_utility::StaircaseDetectorResult::StairsDetectedUp;
    }

    mark_detection_ = false;
    mark_detection_ = this->searchForDescendingStairs();
    if(mark_detection_){
        this->populateDescendingStairs(stair_down);
        if(result == stair_utility::StaircaseDetectorResult::StairsDetectedUp)
            result = stair_utility::StaircaseDetectorResult::StairsDetectedBoth;
        else
            result = stair_utility::StaircaseDetectorResult::StairsDetectedDown;
    }
    return result;
}   

void StairDetector::SegmentPointCloud(){

    int laserCloudSize = voxelgrid_input_->points.size();
    int indZ, indT, indX, indY;
    pcl::PointXYZI point;

    lasercloud_projected_->points.clear();
    lasercloud_projected_->points.resize(cloud_length_ * cloud_width_);

    lasercloud_cylindrical_->points.clear();
    lasercloud_cylindrical_->points.resize(cloud_ang_width_ * cloud_z_height_);   

    for (long i = 0; i < laserCloudSize; i++) {

        point = voxelgrid_input_->points[i];

        float pointX = point.x;
        float pointY = point.y; 
        float pointZ = point.z;
        indX = floor((pointX - x_min_) * cloud_length_ /(x_max_ - x_min_));
        indY = floor((pointY - y_min_) * cloud_width_ / (y_max_ - y_min_));
            
        if( indX >=0  && indX < cloud_length_ && indY >= 0 && indY < cloud_width_){
            if(lasercloud_projected_->at(indY, indX).intensity > 0){
                float pointZd = lasercloud_projected_->at(indY, indX).z;
                if(pointZ > (pointZd + leaf_size_)){
                    // if(pointZ > pointZ_){
                        lasercloud_projected_->at(indY, indX).x = pointX;
                        lasercloud_projected_->at(indY, indX).y = pointY;
                        lasercloud_projected_->at(indY, indX).z = pointZ > pointZd ? pointZ : pointZd;
                        lasercloud_projected_->at(indY, indX).intensity = 1;
                    // }
                    // else{
                    //     laserCloudProjected->at(indY, indX).x = pointX;
                    //     laserCloudProjected->at(indY, indX).y = pointY;
                    //     laserCloudProjected->at(indY, indX).z = pointZ_;
                    //     laserCloudProjected->at(indY, indX).intensity = 1;
                    // }
                }
            }
            else{
                    lasercloud_projected_->at(indY, indX).x = pointX;
                    lasercloud_projected_->at(indY, indX).y = pointY;
                    lasercloud_projected_->at(indY, indX).z = pointZ;
                    lasercloud_projected_->at(indY, indX).intensity = 1;
            }
        }
    }

    for(int i = 0; i < cloud_length_; i++ ){
        for(int j = 0; j < cloud_width_; j++ ){
            point = lasercloud_projected_->at(j,i);
            float pointX = point.x;
            float pointY = point.y; 
            float pointZ = point.z;
            
            if(point.intensity > 0){
                float curr_dist = sqrt(pointX*pointX + pointY*pointY);
                indZ = round((pointZ - z_min_) * cloud_z_height_ /(z_max_ - z_min_));
                indT = floor( (((180 * atan2(pointY, pointX) / PI) + 180) * cloud_ang_width_)/(360));
                float z_offset = pointZ + robot_height_;
                
                if( indZ >=0  && indZ < cloud_z_height_ && indT >= 0 && indT < cloud_ang_width_ && curr_dist >= 0.1){
                    if(lasercloud_cylindrical_->at(indT, indZ).intensity > 0){
                        float pointX_ = lasercloud_cylindrical_->at(indT, indZ).x;
                        float pointY_ = lasercloud_cylindrical_->at(indT, indZ).y;
                        float pointZ_ = lasercloud_cylindrical_->at(indT, indZ).z;
                        float dist = sqrt(pointX_*pointX_ + pointY_*pointY_);
                        if(dist >= curr_dist && z_offset >= 0){
                            lasercloud_cylindrical_->at(indT, indZ).x = pointX;
                            lasercloud_cylindrical_->at(indT, indZ).y = pointY;
                            lasercloud_cylindrical_->at(indT, indZ).z = pointZ;
                            lasercloud_cylindrical_->at(indT, indZ).intensity = indZ+1;
                        }
                        if(dist < curr_dist && z_offset < 0){
                            lasercloud_cylindrical_->at(indT, indZ).x = pointX;
                            lasercloud_cylindrical_->at(indT, indZ).y = pointY;
                            lasercloud_cylindrical_->at(indT, indZ).z = pointZ;
                            lasercloud_cylindrical_->at(indT, indZ).intensity = indZ+1;
                        }
                    }
                    else{
                        lasercloud_cylindrical_->at(indT, indZ).x = pointX;
                        lasercloud_cylindrical_->at(indT, indZ).y = pointY;
                        lasercloud_cylindrical_->at(indT, indZ).z = pointZ;
                        lasercloud_cylindrical_->at(indT, indZ).intensity = indZ+1;
                    }
                }
            }
            
        }
    }
    
}

void StairDetector::getLinesFromCloud(){

    pcl::PointXYZI point, point2;
    int linecount = 0, pointcount = 0;

    std::vector<float> ranges, xs, ys, zs;
    bool point_initialized, slope_initialized;

    int check1 =0, check2 =0;
    for(int i = 0; i <  cloud_z_height_; i++){
        float z_mid = (i * (z_max_ - z_min_) / cloud_z_height_) + z_min_ + leaf_size_/2;
        ranges.clear();
        xs.clear();
        ys.clear();
        zs.clear();
        for(int j = 0; j < cloud_ang_width_; j++){
            point = lasercloud_cylindrical_->at(j,i);
            //If Point is not empty
            if(point.intensity > 0){
                xs.push_back(point.x);
                ys.push_back(point.y);
                zs.push_back(point.z);
                float range = sqrt(pow(point.x, 2) + pow(point.y, 2));
                ranges.push_back(range);
                check1++;
            }
            else{
                xs.push_back(0);
                ys.push_back(0);
                zs.push_back(0);
                ranges.push_back(0.0);
                check2++;
            }       
        }
        line_extractor_.setRangeData(ranges, xs, ys, zs);
        // std::vector<Line> lines;
        // line_extractor_.extractLines(lines);
        if(i < ground_index_ - ground_line_padding_size_){
            line_extractor_.extractLines(detected_lines_below_[i]);
            filterLowQualityLines(detected_lines_below_[i]);
            linecount = linecount + detected_lines_below_[i]->size();
        }
        else if(i >= (ground_index_ - ground_line_padding_size_) && i <= (ground_index_ + ground_line_padding_size_)){
            line_extractor_.extractLines(detected_lines_ground_[i - ground_index_ + ground_line_padding_size_]);
            filterLowQualityLines(detected_lines_ground_[i - ground_index_ + ground_line_padding_size_]);
            linecount = linecount + detected_lines_ground_[i - ground_index_ + ground_line_padding_size_]->size();
        }
        else{
            line_extractor_.extractLines(detected_lines_above_[i - ground_index_ - ground_line_padding_size_ - 1 ]);
            filterLowQualityLines(detected_lines_above_[i - ground_index_ - ground_line_padding_size_ - 1 ]);
            linecount = linecount + detected_lines_above_[i - ground_index_ - ground_line_padding_size_ - 1]->size();
        }
    }
    if(check1 == 0)
        std::cout << "\033[0;33m[Stair Detector]processed cloud is empty: error! \033[0m" << std::endl;

}

// Drop lines that are too far from the robot, or whose fit is too uncertain to be trusted.
// Operating on the deque (rather than the per-line 'skip' flag) guarantees the rejected
// lines are excluded from every stage of staircase search, not just initialization.
void StairDetector::filterLowQualityLines(const std::shared_ptr<std::deque<stair_utility::DetectedLine>>& lines){
    if(max_detection_range_ <= 0.0 && max_line_fit_stddev_ <= 0.0)
        return; // both gates disabled

    auto reject = [this](const stair_utility::DetectedLine& l){
        // Range gate: distance from the robot (local-frame origin) to the line center.
        if(max_detection_range_ > 0.0){
            double range = std::sqrt(l.line_center[0] * l.line_center[0] + l.line_center[1] * l.line_center[1]);
            if(range > max_detection_range_)
                return true;
        }
        // Fit-quality gate: standard deviation of the fitted line radius (line_covariance[0] is the radius variance).
        if(max_line_fit_stddev_ > 0.0){
            if(std::sqrt(std::fabs(l.line_covariance[0])) > max_line_fit_stddev_)
                return true;
        }
        return false;
    };

    lines->erase(std::remove_if(lines->begin(), lines->end(), reject), lines->end());
}

// A genuine staircase has regular geometry: consecutive steps are evenly spaced in depth
// and rise by a consistent height. Reject candidates whose inter-step depth/height varies
// too much (a common signature of spurious groupings of unrelated lines).
bool StairDetector::isStaircaseConsistent(const std::vector<stair_utility::DetectedLine>& steps) const {
    if(max_step_depth_variation_ <= 0.0 && max_step_height_variation_ <= 0.0)
        return true; // both gates disabled

    if(steps.size() < 3)
        return true; // need at least two gaps to assess regularity

    std::vector<double> depths, heights;
    depths.reserve(steps.size() - 1);
    heights.reserve(steps.size() - 1);
    for(size_t i = 1; i < steps.size(); ++i){
        depths.push_back(stair_utility::get_xy_distance(steps[i].line_center, steps[i - 1].line_center));
        heights.push_back(std::fabs(steps[i].line_center[2] - steps[i - 1].line_center[2]));
    }

    auto stddev = [](const std::vector<double>& v){
        double mean = 0.0;
        for(double x : v) mean += x;
        mean /= v.size();
        double var = 0.0;
        for(double x : v) var += (x - mean) * (x - mean);
        return std::sqrt(var / v.size());
    };

    if(max_step_depth_variation_ > 0.0 && stddev(depths) > max_step_depth_variation_){
        std::cout << "\033[1;33m[Stair Detector] Staircase rejected: irregular step depth (std " << stddev(depths)
                  << " > " << max_step_depth_variation_ << ") \033[0m" << std::endl;
        return false;
    }
    if(max_step_height_variation_ > 0.0 && stddev(heights) > max_step_height_variation_){
        std::cout << "\033[1;33m[Stair Detector] Staircase rejected: irregular step height (std " << stddev(heights)
                  << " > " << max_step_height_variation_ << ") \033[0m" << std::endl;
        return false;
    }
    return true;
}

bool StairDetector::searchForAscendingStairs(){
    // Detect Stairs going up
    stair_detected_ = false;    
    bool line_initialized = false, is_init_space_empty = false, stair_inited = false;
    int initial_step_index, second_step_index;
    stair_utility::DetectedLine line1, line2, ramp_l;
    int line2_index, line1_index;
    int might_be_slope;
    //Group Lines that are close together 
    //Main search space.
    while(!stair_detected_ && !is_init_space_empty){
        stairs_up_.clear();
        ramp_lines_up_.clear();
        stair_inited = false;
        might_be_slope = 0;
        //(Initialization will be from Init space. Get 2 lines that look like a stair - )
        for(int ind = 0; ind < detected_lines_above_.size(); ind++){
            std::shared_ptr<std::deque<stair_utility::DetectedLine>> curr_lines = detected_lines_above_[ind];
            // std::cout << " [Stair Detector] Current lines: " << curr_lines->size() << std::endl;
            if(ind > stair_initialization_range_){
                // std::cout << " [Stair Detector] No initializations found" << std::endl;
                break;
            }
            
            if(stair_inited){
                //  std::cout << " [Stair Detector] INITIEDDD" << std::endl;
                break;
            }
            
            if(curr_lines->size() > 0){
                if(!line_initialized){
                    line1_index = 0;
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                        line1 = *c;
                        if(!line1.skip){
                            line_initialized = true;
                            initial_step_index = ind;
                            // std::cout << "[Stair Detector] Line inited" << std::endl;
                            break;
                        }
                        line1_index++;
                    }
                    
                }
                else{
                    line2_index = 0;
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                        line2 = *c;
                        if(!line2.skip){
                            float center_diff = stair_utility::get_xy_distance(line2.line_center, line1.line_center);
                            float line_diff = stair_utility::get_line_distance(line1.line_start, line1.line_end, line2.line_center);
                            float angle_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, line1.line_yaw_angle);
                            // float length_diff = fabs(line2.line_length - line1.line_length);
                            float relative_length_diff = fabs(line2.line_length - line1.line_length) / line1.line_length;
                            float z_diff = line2.line_center[2] - line1.line_center[2];
                            float yaw_slope = atan2(z_diff, center_diff);
                            bool valid_dist = center_diff >= min_stair_depth_ && center_diff <= max_stair_depth_ && line_diff >= min_stair_depth_ && line_diff <= max_stair_depth_ && fabs(line_diff - center_diff) <= 0.05;
                            bool valid_slope = (yaw_slope >= stair_slope_min_ && yaw_slope <= stair_slope_max_);
                            // std::cout << " [Stair Detector] line d: " << line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;
                            if(valid_dist && angle_diff < max_stair_curvature_ && relative_length_diff <= 0.3 && z_diff >= min_stair_height_ && z_diff <= max_stair_height_ && valid_slope){
                                
                                if(ramp_lines_up_.size() == 0){
                                    stair_inited = true;
                                    stairs_up_.push_back(line1);
                                    stairs_up_.push_back(line2);
                                    detected_lines_above_[ind]->at(line2_index).skip = true;
                                    // std::cout << "[Stair Detector] Stair Inited" << std::endl;
                                    second_step_index = ind;
                                    break;
                                }
                                else{
                                    ramp_l = ramp_lines_up_.back();
                                    float cent_ramp = stair_utility::get_xy_distance(line2.line_center, ramp_l.line_center);
                                    float lineR_diff = stair_utility::get_line_distance(ramp_l.line_start, ramp_l.line_end, line2.line_center);
                                    float zR_diff =  line2.line_center[2] - ramp_l.line_center[2];
                                    float angleR_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, ramp_l.line_yaw_angle);
                                    //std::cout << "[Stair Detector] line d: " << lineR_diff <<" d: " << cent_ramp <<" an: "<< angleR_diff <<" z_diff: "<< zR_diff << std::endl;
                                    //std::cout << "[Stair Detector] R line d: " << line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;

                                    bool valid_dist_ramp = lineR_diff < 0.15 && zR_diff < 0.3 && angleR_diff < 0.55;
                                    if(!valid_dist_ramp){
                                        stair_inited = true;
                                        stairs_up_.push_back(line1);
                                        stairs_up_.push_back(line2);
                                        detected_lines_above_[ind]->at(line2_index).skip = true;
                                        //std::cout << ("[Stair Detector] Stair Inited");
                                        second_step_index = ind;
                                        break;
                                    }
                                    else{
                                        if(use_ramp_detection_){
                                            ramp_lines_up_.push_back(line2);
                                            might_be_slope++;
                                            //std::cout << (" [Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_up.size());
                                            break;
                                        }
                                    }
                                }
                                
                            }
                            if(use_ramp_detection_){
                                if(z_diff > 0.025 && z_diff <= 0.3){
                                    if(line_diff < 0.15 && center_diff < 0.15 && angle_diff < 0.4 && fabs(line_diff - center_diff) <= 0.1){
                                        //Check for a ramp     
                                        might_be_slope++;
                                        ramp_lines_up_.push_back(line2);
                                        //std::cout << ("Might be ramp Line: " << might_be_slope  << " " << ramp_up.size());
                                    }
                                    if(valid_dist && angle_diff < 0.40){
                                        might_be_slope++;
                                        ramp_lines_up_.push_back(line2);
                                        //std::cout << ("Might be ramp Line: " << might_be_slope  << " " << ramp_up.size());
                                    }
                                }
                            }
                        }
                        line2_index++;
                    }
                }
            }   
        }
        
        if(stair_inited){
            for(int ind = second_step_index+1; ind < detected_lines_above_.size(); ind++){
                std::shared_ptr<std::deque<stair_utility::DetectedLine>> curr_lines = detected_lines_above_[ind];
                line1 = stairs_up_.back();
                if(curr_lines->size() > 0){
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                            line2 = *c;
                            float center_diff = stair_utility::get_xy_distance(line2.line_center, line1.line_center);
                            float line_diff = stair_utility::get_line_distance(line1.line_start, line1.line_end, line2.line_center);
                            float angle_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, line1.line_yaw_angle);
                            // float length_diff = fabs(line2.line_length - line1.line_length);
                            float relative_length_diff = fabs(line2.line_length - line1.line_length) / line1.line_length;
                            float z_diff = fabs(line2.line_center[2] - line1.line_center[2]);
                            float yaw_slope = atan2(z_diff, center_diff);
                            bool valid_dist = center_diff >= min_stair_depth_ && center_diff <= max_stair_depth_ && line_diff >= min_stair_depth_ && line_diff <= max_stair_depth_ && fabs(line_diff - center_diff) <= 0.05;
                            bool valid_slope = (yaw_slope >= stair_slope_min_ && yaw_slope <= stair_slope_max_);
                            // std::cout << "[Stair Detector] line d: "<< line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;
                            if(valid_dist && angle_diff < max_stair_curvature_ && relative_length_diff <= 0.3 && z_diff >= min_stair_height_ && z_diff <= max_stair_height_ && valid_slope){
                                if(ramp_lines_up_.size() == 0){
                                    stair_utility::DetectedLine line0 = stairs_up_[0];
                                    float center_0_diff = stair_utility::get_xy_distance(line2.line_center, line0.line_center);
                                    if(center_0_diff > center_diff){
                                        stairs_up_.push_back(line2);
                                        // std::cout << " [Stair Detector] Next step found" << std::endl;
                                        break;
                                    }
                                }
                                else{
                                    ramp_l = ramp_lines_up_.back();
                                    float cent_ramp = stair_utility::get_xy_distance(line2.line_center, ramp_l.line_center);
                                    float lineR_diff = stair_utility::get_line_distance(ramp_l.line_start, ramp_l.line_end, line2.line_center);
                                    float zR_diff =  line2.line_center[2] - ramp_l.line_center[2];
                                    float angleR_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, ramp_l.line_yaw_angle);
                                    bool valid_dist_ramp =  lineR_diff < 0.15 && zR_diff < 0.11 && angleR_diff < 0.25;
                                    // std::cout << " [Stair Detector] line d: " << lineR_diff <<" d: " << cent_ramp <<" an: "<< angleR_diff <<" z_diff: "<< zR_diff << std::endl;
                                    // std::cout << " [Stair Detector] R line d: " << line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;
                                    if(!valid_dist_ramp){
                                        stair_utility::DetectedLine line0 = stairs_up_[0];
                                        float center_0_diff = stair_utility::get_xy_distance(line2.line_center, line0.line_center);
                                        if(center_0_diff > center_diff){
                                            stairs_up_.push_back(line2);
                                            // std::cout << "[Stair Detector] Next step found" << std::endl;
                                            break;
                                        }
                                    }
                                    else{
                                        if(use_ramp_detection_){
                                            ramp_lines_up_.push_back(line2);
                                            might_be_slope++;
                                            // std::cout << "[Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_lines_up_.size() << std::endl;
                                            break;
                                        }
                                    }
                                }
                            }

                            //Check for a similar line
                            if(fabs(z_diff) <= (1.2*leaf_size_)){
                                if(line_diff <= 0.15 && center_diff <= 0.15 && angle_diff < 0.2 && fabs(line_diff - center_diff) <= 0.05 && relative_length_diff < 0.2){
                                    //Similar line found,  just pick the best line and replace.
                                    std::array<double,3 > cent1 = line1.line_center;
                                    std::array<double,3 > cent2 = line2.line_center;
                                    float range1 = sqrt(pow(cent1[0], 2 ) + pow(cent1[1], 2));
                                    float range2 = sqrt(pow(cent2[0], 2 ) + pow(cent2[1], 2));
                                    // std::cout << " [Stair Detector] range_diff: " << range1 - range2 << "l2 len: " << line2.line_length << "l1 le: " << line1.line_length << std::endl;
                                    //if new closer line is atleast 10cms longer than old one 
                                    if((line2.line_length - line1.line_length > -0.05)){
                                        // range1 > range2 means new line is closer
                                        if(range1 - range2 >= 0 && range1 - range2 <= 0.1){
                                            // std::cout << "[Stair Detector] Similar Line found" << std::endl;
                                            stairs_up_.pop_back();
                                            stairs_up_.push_back(line2);
                                            break;
                                        } 
                                    }
                                }
                            }
                            else{
                                if(use_ramp_detection_){
                                    if(line_diff < 0.15 && center_diff < 0.15 && angle_diff < 0.4 && fabs(line_diff - center_diff) <= 0.1 && z_diff <= 0.3){
                                        might_be_slope++; 
                                        ramp_lines_up_.push_back(line2);
                                        // std::cout << "[Stair Detector] Might be ramp Line: " << might_be_slope  << " " << ramp_lines_up_.size() << std::endl;
                                    }

                                    if(valid_dist && angle_diff < 0.40 && z_diff < 0.11){
                                        might_be_slope++;
                                        ramp_lines_up_.push_back(line2);
                                        // std::cout << "[Stair Detector] Might be ramp Line: " << might_be_slope  << " " << ramp_lines_up_.size() << std::endl;
                                    }
                                }
                            }  
                    }
                }
            }
            // std::cout << "[Stair Detector ]Num stair in loop " << stairs_up.size() << std::endl;
            if(stairs_up_.size() >= min_stair_count_){
                if(ramp_lines_up_.size() < ceil((int)stairs_up_.size())){
                    if(isStaircaseConsistent(stairs_up_)){
                        stair_detected_ = true;
                    }
                    break;
                }
                else{
                    std::cout << "\033[1;33m[Stair Detector] Staircase Rejected due to probable ramp: " << ramp_lines_up_.size() << "/" << stairs_up_.size() << " \033[0m"<< std::endl;
                    break;
                }
            }
            
            if(line_initialized && !stair_detected_){
                line_initialized = false;
            }
        }

        if(!line_initialized && !stair_detected_ && !stair_inited)
                is_init_space_empty = true;

        if(line_initialized && !stair_inited){
            line_initialized = false;
            // std::cout << ("Deleting start line");
            detected_lines_above_[initial_step_index]->at(line1_index).skip = true;
        }
    }
    if(stair_detected_){
        std::cout << "\033[1;32m[Stair Detector] Staircase Detected Going Up with " << stairs_up_.size() << " steps! \033[0m" << std::endl;
        return true;
    }

    return false;
}

bool StairDetector::searchForDescendingStairs(){
    // Detect Stairs going down
    stair_detected_ = false;    
    bool line_initialized = false, is_init_space_empty = false, stair_inited = false;
    int initial_step_index, second_step_index;
    stair_utility::DetectedLine line1, line2, ramp_l;
    int line2_index, line1_index;
    int might_be_slope;
    while(!stair_detected_ && !is_init_space_empty){
        stairs_down_.clear();
        ramp_lines_down_.clear();
        stair_inited = false;
        might_be_slope = 0;
        //(Initialization will be from Init space. Get 2 lines that look like a stair - )
        for(int ind = detected_lines_below_.size() - 1; ind >= 0; ind--){
            std::shared_ptr<std::deque<stair_utility::DetectedLine>> curr_lines = detected_lines_below_[ind];
            // std::cout << "[Stair Detector] Current lines: " << curr_lines->size() << std::endl;
            if(ind < ground_index_ - stair_initialization_range_){
                // std::cout << " [Stair Detector] No initializations found" << std::endl;
                break;
            }
            
            if(stair_inited){
                break;
            }
            
            if(curr_lines->size() > 0){
                if(!line_initialized){
                    line1_index = 0;
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                        line1 = *c;
                        if(!line1.skip){
                            line_initialized = true;
                            initial_step_index = ind;
                            //std::cout << "[Stair Detector] Line inited" << std::endl;
                            break;
                        }
                        line1_index++;
                    }
                    
                }
                else{
                    line2_index = 0;
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                        line2 = *c;
                        if(!line2.skip){
                            float center_diff = stair_utility::get_xy_distance(line2.line_center, line1.line_center);
                            float line_diff = stair_utility::get_line_distance(line1.line_start, line1.line_end, line2.line_center);
                            float angle_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, line1.line_yaw_angle);
                            // float length_diff = fabs(line2.line_length - line1.line_length);
                            float relative_length_diff = fabs(line2.line_length - line1.line_length) / line1.line_length;
                            float z_diff = fabs(line2.line_center[2] - line1.line_center[2]);
                            float yaw_slope = atan2(fabs(z_diff), center_diff);
                            bool valid_dist = center_diff >= min_stair_depth_ && center_diff <= max_stair_depth_ && line_diff >= min_stair_depth_ && line_diff <= max_stair_depth_ && fabs(line_diff - center_diff) <= 0.05;
                            bool valid_slope = (yaw_slope >= stair_slope_min_ && yaw_slope <= stair_slope_max_);
                            // std::cout << "[Stair Detector] line d: " << line_diff << " d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << std::endl;
                            if(valid_dist && angle_diff < max_stair_curvature_ && relative_length_diff <= 0.3 && z_diff >= min_stair_height_ && z_diff <= max_stair_height_ && valid_slope){
                                
                                if(ramp_lines_down_.size() == 0){
                                    stair_inited = true;
                                    stairs_down_.push_back(line1);
                                    stairs_down_.push_back(line2);
                                    detected_lines_below_[ind]->at(line2_index).skip = true;
                                    //std::cout << "[Stair Detector] Stair Inited" << std::endl;
                                    second_step_index = ind;
                                    break;
                                }
                                else{
                                    ramp_l = ramp_lines_down_.back();
                                    float cent_ramp = stair_utility::get_xy_distance(line2.line_center, ramp_l.line_center);
                                    float lineR_diff = stair_utility::get_line_distance(ramp_l.line_start, ramp_l.line_end, line2.line_center);
                                    float zR_diff =  line2.line_center[2] - ramp_l.line_center[2];
                                    float angleR_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, ramp_l.line_yaw_angle);
                                    // std::cout << "[Stair Detector] line d: " << lineR_diff <<" d: " << cent_ramp <<" an: "<< angleR_diff <<" z_diff: "<< zR_diff << std::endl;
                                    // std::cout << "[Stair Detector] R line d: " << line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;

                                    bool valid_dist_ramp = lineR_diff < 0.15 && zR_diff < 0.3 && angleR_diff < 0.55;
                                    if(!valid_dist_ramp){
                                        stair_inited = true;
                                        stairs_down_.push_back(line1);
                                        stairs_down_.push_back(line2);
                                        detected_lines_below_[ind]->at(line2_index).skip = true;
                                        // std::cout << "[Stair Detector] Stair Inited" << std::endl;
                                        second_step_index = ind;
                                        break;
                                    }
                                    else{
                                        if(use_ramp_detection_){
                                            ramp_lines_down_.push_back(line2);
                                            might_be_slope++;
                                            // std::cout << "[Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                            break;
                                        }
                                    }
                                }
                            }
                            if(use_ramp_detection_){
                                if(z_diff > 0.025 && z_diff <= 0.3){
                                    if(line_diff < 0.15 && center_diff < 0.15 && angle_diff < 0.4 && fabs(line_diff - center_diff) <= 0.1){
                                        //Check for a ramp     
                                        might_be_slope++;
                                        ramp_lines_down_.push_back(line2);
                                        //std::cout << "[Stair Detector] Might be ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                    }
                                    if(valid_dist && angle_diff && angle_diff < 0.40){
                                        might_be_slope++;
                                        ramp_lines_down_.push_back(line2);
                                        //std::cout << "[Stair Detector] Might be ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                    }
                                }
                            }
                        }
                        line2_index++;
                    }
                }
            }   
        }
        
        if(stair_inited){
            for(int ind = second_step_index-1; ind >= 0 ; ind--){
                std::shared_ptr<std::deque<stair_utility::DetectedLine>> curr_lines = detected_lines_below_[ind];
                line1 = stairs_down_.back();
                if(curr_lines->size() > 0){
                    for(std::deque<stair_utility::DetectedLine>::const_iterator c = curr_lines->begin(); c != curr_lines->end(); ++c){
                            line2 = *c;
                            float center_diff = stair_utility::get_xy_distance(line2.line_center, line1.line_center);
                            float line_diff = stair_utility::get_line_distance(line1.line_start, line1.line_end, line2.line_center);                            
                            float angle_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, line1.line_yaw_angle);
                            // float length_diff = fabs(line2.line_length - line1.line_length);
                            float relative_length_diff = fabs(line2.line_length - line1.line_length) / line1.line_length;
                            float z_diff = fabs(line2.line_center[2] - line1.line_center[2]);
                            float yaw_slope = atan2(fabs(z_diff), center_diff);
                            bool valid_dist = center_diff >= min_stair_depth_ && center_diff <= max_stair_depth_ && line_diff >= min_stair_depth_ && line_diff <= max_stair_depth_ && fabs(line_diff - center_diff) <= 0.05;                            
                            bool valid_slope = (yaw_slope >= stair_slope_min_ && yaw_slope <= stair_slope_max_);
                            //std::cout << "[Stair Detector] line d: "<< line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << std::endl;
                            if(valid_dist && angle_diff < max_stair_curvature_ && relative_length_diff <= 0.3 && z_diff >= min_stair_height_ && z_diff <= max_stair_height_ && valid_slope){
                                if(ramp_lines_down_.size() == 0){
                                    stair_utility::DetectedLine line0 = stairs_down_[0];
                                    float center_0_diff = stair_utility::get_xy_distance(line2.line_center, line0.line_center);
                                    if(center_0_diff > center_diff){
                                        stairs_down_.push_back(line2);
                                        //std::cout << "[Stair Detector] Next step found" << std::endl;
                                        break;
                                    }
                                }
                                else{
                                    ramp_l = ramp_lines_down_.back();
                                    float cent_ramp = stair_utility::get_xy_distance(line2.line_center, ramp_l.line_center);
                                    float lineR_diff = stair_utility::get_line_distance(ramp_l.line_start, ramp_l.line_end, line2.line_center);
                                    float zR_diff =  line2.line_center[2] - ramp_l.line_center[2];
                                    float angleR_diff = stair_utility::get_line_angle_diff(line2.line_yaw_angle, ramp_l.line_yaw_angle);
                                    bool valid_dist_ramp =  lineR_diff < 0.15 && zR_diff < 0.11 && angleR_diff < 0.25;
                                    //std::cout << "[Stair Detector] line d: " << lineR_diff <<" d: " << cent_ramp <<" an: "<< angleR_diff <<" z_diff: "<< zR_diff << std::endl;
                                    //std::cout << "[Stair Detector] R line d: " << line_diff <<" d: " << center_diff <<" an: "<< angle_diff <<" ldp: "<< relative_length_diff <<" z_diff: "<< z_diff << "yaw Slo: " << yaw_slope << std::endl;
                                    if(!valid_dist_ramp){
                                        stair_utility::DetectedLine line0 = stairs_down_[0];
                                        float center_0_diff = stair_utility::get_xy_distance(line2.line_center, line0.line_center);
                                        if(center_0_diff > center_diff){
                                            stairs_down_.push_back(line2);
                                            //std::cout << "[Stair Detector] Next step found" << std::endl;
                                            break;
                                        }
                                    }
                                    else{
                                        if(use_ramp_detection_){
                                            ramp_lines_down_.push_back(line2);
                                            might_be_slope++;
                                            //std::cout << "[Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                            break;
                                        }
                                    }
                                }
                                
                            }
                            
                            //Check for a similar line
                            if(fabs(z_diff) <= (1.2*leaf_size_)){
                                if(line_diff <= 0.05 && center_diff <= 0.05 && angle_diff < 0.2 && fabs(line_diff - center_diff) <= 0.05 && relative_length_diff < 0.2){
                                    //Similar line found,  just pick the best line and replace.
                                    std::array<double,3 > cent1 = line1.line_center;
                                    std::array<double,3 > cent2 = line2.line_center;
                                    float range1 = sqrt(pow(cent1[0], 2 ) + pow(cent1[1], 2));
                                    float range2 = sqrt(pow(cent2[0], 2 ) + pow(cent2[1], 2));
                                    //if new closer line is atleast 10cms longer than old one
                                    // std::cout << "[Stair Detector] range_diff: " << range1 - range2 << "l2 len: " << line2.line_length << "l1 le: " << line1.line_length << std::endl;
                                    if((line2.line_length - line1.line_length > -0.1)){
                                        // range1 > range2 means new line is closer
                                        if(range1 - range2 >= -0.05 && range1 - range2 <= 0.1){
                                            //std::cout << "Similar Line found" << std::endl;
                                            stairs_down_.pop_back();
                                            stairs_down_.push_back(line2);
                                            break;
                                        } 
                                    }
                                }
                            }
                            else{
                                if(use_ramp_detection_){
                                    if(line_diff < 0.15 && center_diff < 0.15 && angle_diff < 0.4 && fabs(line_diff - center_diff) <= 0.1 && z_diff <= 0.3){
                                        might_be_slope++;
                                        ramp_lines_down_.push_back(line2);
                                        //std::cout << "[Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                    }

                                    if(valid_dist && angle_diff && angle_diff < 0.40 && z_diff < 0.11){
                                        might_be_slope++;
                                        ramp_lines_down_.push_back(line2);
                                        //std::cout << "[Stair Detector] Ramp Line: " << might_be_slope << " " << ramp_lines_down_.size() << std::endl;
                                    }
                                }
                            }
                    }
                }
            }
            // std::cout << "[Stair Detector] Num stair in loop " << stairs_down_.size() << std::endl;
            if(stairs_down_.size() >= min_stair_count_){
                if(ramp_lines_down_.size() < ceil((int)stairs_down_.size())){
                    if(isStaircaseConsistent(stairs_down_)){
                        stair_detected_ = true;
                    }
                    break;
                }
                else{
                    std::cout << "\033[1;33m[Stair Detector] Staircase Rejected due to probable ramp: " << ramp_lines_down_.size() << "/" << stairs_down_.size() << " \033[0m" << std::endl;
                    break;
                }
            }
            
            if(line_initialized && !stair_detected_){
                line_initialized = false;
            }
        }

        if(!line_initialized && !stair_detected_ && !stair_inited)
                is_init_space_empty = true;

        if(line_initialized && !stair_inited){
            line_initialized = false;
            detected_lines_below_[initial_step_index]->at(line1_index).skip = true;
        }
    }
     if(stair_detected_){
        std::cout << "\033[1;32m[Stair Detector] Staircase Detected Going Down " << stairs_down_.size() << " steps! \033[0m" << std::endl;
        return true;
    }

    return false;
}

void StairDetector::populateAscendingStairs(stair_utility::StaircaseMeasurement& stair_up){
    
    stair_up.stair_count = stairs_up_.size();
    stair_up.steps.clear();
    for(std::vector<stair_utility::DetectedLine>::const_iterator c = stairs_up_.begin(); c != stairs_up_.end(); ++c){
        stair_utility::StairStep step;
            
        step.start_p(0) = static_cast<float>(c->line_start[0]);
        step.start_p(1) = static_cast<float>(c->line_start[1]);
        step.start_p(2) = static_cast<float>(c->line_start[2]);
                    
        step.end_p(0) = static_cast<float>(c->line_end[0]);
        step.end_p(1) = static_cast<float>(c->line_end[1]);
        step.end_p(2) = static_cast<float>(c->line_end[2]);

        step.step_width = c->line_length;

        step.line_polar_form(0) = c->line_radius;
        step.line_polar_form(1) = c->line_theta;

        step.step_covariance.setZero();
        step.step_covariance(0, 0) = c->line_covariance[0];
        step.step_covariance(0, 1) = c->line_covariance[1];
        step.step_covariance(1, 0) = c->line_covariance[2];
        step.step_covariance(1, 1) = c->line_covariance[3];
        step.step_covariance(2, 2) = c->line_z_variance;
        step.step_covariance(3, 3) = c->line_z_variance;

        stair_up.steps.push_back(step);
    }                  
}

void StairDetector::populateDescendingStairs(stair_utility::StaircaseMeasurement& stair_down){
    
    stair_down.stair_count = stairs_down_.size();
    stair_down.steps.clear();

    for(std::vector<stair_utility::DetectedLine>::const_reverse_iterator c = stairs_down_.rbegin(); c != stairs_down_.rend(); ++c){
        stair_utility::StairStep step;
            
        step.start_p(0) = static_cast<float>(c->line_start[0]);
        step.start_p(1) = static_cast<float>(c->line_start[1]);
        step.start_p(2) = static_cast<float>(c->line_start[2]);
                    
        step.end_p(0) = static_cast<float>(c->line_end[0]);
        step.end_p(1) = static_cast<float>(c->line_end[1]);
        step.end_p(2) = static_cast<float>(c->line_end[2]);
    
        step.step_width = c->line_length;

        step.line_polar_form(0) = c->line_radius;
        step.line_polar_form(1) = c->line_theta;

        step.step_covariance.setZero();
        step.step_covariance(0, 0) = c->line_covariance[0];
        step.step_covariance(0, 1) = c->line_covariance[1];
        step.step_covariance(1, 0) = c->line_covariance[2];
        step.step_covariance(1, 1) = c->line_covariance[3];
        step.step_covariance(2, 2) = c->line_z_variance;
        step.step_covariance(3, 3) = c->line_z_variance;
        
        stair_down.steps.push_back(step);
    }                  
}

void StairDetector::getProcessedCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr processed, int stage){
    if(stage == 1)
        *processed = *lasercloud_projected_;
    else if(stage == 2)
        *processed = *lasercloud_cylindrical_;
}
