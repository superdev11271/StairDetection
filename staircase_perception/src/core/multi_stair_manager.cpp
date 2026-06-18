#include "staircase_perception/core/multi_stair_manager.hpp"

/* Implementations of the Base Staircase Manager class */

int BaseStaircaseManager::getNewId(){

    int new_id = number_of_stairs_ + 1;
    while(ids_.find(new_id) != ids_.end()){
        new_id++;
    }
    return new_id;
}

bool BaseStaircaseManager::areLinesIntersecting(const Eigen::Vector3d &pointA, const Eigen::Vector3d &pointB, const Eigen::Vector3d &pointC, const Eigen::Vector3d &pointD){

    float a = ((pointD(0) - pointC(0))*(pointC(1) - pointA(1))) - ((pointD(1) - pointC(1))*(pointC(0) - pointA(0)));
    float b = ((pointD(0) - pointC(0))*(pointB(1) - pointA(1))) - ((pointD(1) - pointC(1))*(pointB(0) - pointA(0)));
    float c = ((pointB(0) - pointA(0))*(pointC(1) - pointA(1))) - ((pointB(1) - pointA(1))*(pointC(0) - pointA(0)));

    if(b == 0 && a != 0)
        return false;
    
    if(a == 0 && b == 0)
        return true;

    float alpha = a/b;
    float beta = c/b;

    if(alpha >=0 && alpha <=1 && beta >= 0 && beta <= 1)
        return true;
    else
        return false;


}

int BaseStaircaseManager::checkStaircaseSimilarity(const stair_utility::StaircaseInfo& curr_stairase_info, const stair_utility::StaircaseInfo& new_staircase_info){
    int score = 0;
    float dir_diff = stair_utility::wrap2PI(curr_stairase_info.staircase_direction - new_staircase_info.staircase_direction);

    // std::cout <<"[Staircase Check] st1_dir: " << curr_stairase_info.staircase_direction << " st_dir:" << new_staircase_info.staircase_direction << " diff: " << dir_diff << std::endl;
    // std::cout << "=======================" << std::endl;
    // std::cout << "Current:" << std::endl;
    // for (const Eigen::Vector3d& point : curr_stairase_info.stair_polygon) {
    //     std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
    // }
    // std::cout << "New:" << std::endl;
    // for (const Eigen::Vector3d& point : new_staircase_info.stair_polygon) {
    //     std::cout << "(" << point.x() << ", " << point.y() << ", " << point.z() << ")" << std::endl;
    // }
    // polygon1 = curr_stairase_info;
    // polygon2 = new_staircase_info;

    if(fabs(dir_diff) < stair_manager_params_.yaw_threshold){
        // Also need to add z-checking before computing line intersections.
        double minZ1 = (curr_stairase_info.stair_polygon[0](2) + curr_stairase_info.stair_polygon[1](2))/2;
        double maxZ1 = (curr_stairase_info.stair_polygon[2](2) + curr_stairase_info.stair_polygon[3](2))/2;
        double minZ2 = (new_staircase_info.stair_polygon[0](2) + new_staircase_info.stair_polygon[1](2))/2;
        double maxZ2 = (new_staircase_info.stair_polygon[2](2) + new_staircase_info.stair_polygon[3](2))/2;
        if(std::max(0.0, std::min(maxZ1, maxZ2) - std::max(minZ1, minZ2)) > 0){
            for(int i = 0; i < 4; ++i){
                int l = (i + 1) % 4;
                
                for(int j = 0; j < 4; ++j){
                    int k = (j + 1) % 4;
                    // std::cout << "p = plot3([" << curr_stairase_info.stair_polygon[i](0) << " , " << curr_stairase_info.stair_polygon[l](0) << "], [" << curr_stairase_info.stair_polygon[i](1) << " , " << curr_stairase_info.stair_polygon[l](1) << "], [" << curr_stairase_info.stair_polygon[i](2) << " , " << curr_stairase_info.stair_polygon[l](2) << " ], 'r');" << std::endl;
                    // std::cout << "p = plot3([" << new_staircase_info.stair_polygon[j](0) << " , " << new_staircase_info.stair_polygon[k](0) << "], [" << new_staircase_info.stair_polygon[j](1) << " , " << new_staircase_info.stair_polygon[k](1) << "], [" << new_staircase_info.stair_polygon[j](2) << " , " << new_staircase_info.stair_polygon[k](2) << " ], 'g');" << std::endl;
                    if(areLinesIntersecting(curr_stairase_info.stair_polygon[i], curr_stairase_info.stair_polygon[l], new_staircase_info.stair_polygon[j], new_staircase_info.stair_polygon[k])){
                        score += 1;
                        // std::cout << "[Staircase Manager] Line Intersection found, classifying as same staircase" << std::endl;
                        break;
                    }
                }

                if(score >= 1)
                    break;
                
            }

            if(score < 1){
                for(int i = 0; i < 4; i++){
                    if(stair_utility::isPointInPolygon(new_staircase_info.stair_polygon[i], curr_stairase_info.stair_polygon) || stair_utility::isPointInPolygon(curr_stairase_info.stair_polygon[i], new_staircase_info.stair_polygon)){
                        score += 1;
                        // std::cout << "[Staircase Manager] Point inside polygon, classifying as same staircase" << std::endl;
                        break;
                    }
                }
            }  
        } 

    }   

    return score;
}

void BaseStaircaseManager::getStairInfoFromCorners(const Eigen::Vector3d &begin_start, const Eigen::Vector3d &begin_end, const Eigen::Vector3d &final_start, const Eigen::Vector3d &final_end, stair_utility::StaircaseInfo& stair_info){
    
    // From the two point of the last stair, ensure the polygon is going in order.  
    float theta1 = atan2(final_end(1) - begin_start(1), final_end(0) - begin_start(0));
    float theta2 = atan2(final_start(1) - begin_start(1), final_start(0) - begin_start(0));
    float theta0 = atan2(begin_end(1) - begin_start(1), begin_end(0) - begin_start(0));

    if(fabs(stair_utility::wrap2PI(theta1 -theta0)) < fabs(stair_utility::wrap2PI(theta2 - theta0))){
        // Inflate the measurement by a small amount
        Eigen::Vector3d offset1 = {0.15 * cos(theta1), 0.15 * sin(theta1), 0.12};
        float theta3 = atan2(final_start(1) - begin_end(1), final_start(0) - begin_end(0));
        Eigen::Vector3d offset2 = {0.15 * cos(theta3), 0.15 * sin(theta3), 0.12};

        // Insert the start and end point of the first staircase.
        stair_info.stair_polygon.push_back(begin_start - offset1);
        stair_info.stair_polygon.push_back(begin_end - offset2);

        stair_info.stair_polygon.push_back(final_end + offset1);
        stair_info.stair_polygon.push_back(final_start + offset2);
    }
    else{

        Eigen::Vector3d offset1 = {0.12 * cos(theta2), 0.12 * sin(theta2), 0.1};
        float theta3 = atan2(final_end(1) - begin_end(1), final_end(0) - begin_end(0));
        Eigen::Vector3d offset2 = {0.12 * cos(theta3), 0.12 * sin(theta3), 0.1};

        // Insert the start and end point of the first staircase.
        stair_info.stair_polygon.push_back(begin_start - offset1);
        stair_info.stair_polygon.push_back(begin_end - offset2);

        stair_info.stair_polygon.push_back(final_start + offset1);
        stair_info.stair_polygon.push_back(final_end + offset2);
    }

    Eigen::Vector3d center_s = (begin_start + begin_end)/2;
    Eigen::Vector3d center_e = (final_start + final_end)/2;
    stair_info.staircase_direction = atan2(center_e(1) - center_s(1), center_e(0) - center_s(0));

}

void SingleRobotStairManager::computeStaircaseInfo(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseInfo& stair_info){
    // Compute the polygon of the staircase, and ensure are in order. Clear any left over data. 
    stair_info.stair_polygon.clear();

    Eigen::Vector3d begin_start, begin_end, final_start, final_end;
    Eigen::Affine3d transform = new_staircase.robot_pose.vehicle_pos * new_staircase.robot_pose.vehicle_quat;

    begin_start = transform * new_staircase.steps[0].start_p;
    begin_end = transform * new_staircase.steps[0].end_p;
    final_start = transform * new_staircase.steps[new_staircase.stair_count - 1].start_p;
    final_end = transform * new_staircase.steps[new_staircase.stair_count - 1].end_p;

    getStairInfoFromCorners(begin_start, begin_end, final_start, final_end, stair_info);
}

SingleRobotStairManager::SingleRobotStairManager(const stair_utility::StairManagerParams& manager_params){
    
    stair_manager_params_.yaw_threshold = manager_params.yaw_threshold;
    stair_manager_params_.filter_type = manager_params.filter_type;
    stair_manager_params_.robot_name = manager_params.robot_name;
    stair_manager_params_.filter_sigmas = manager_params.filter_sigmas;
    stair_manager_params_.min_detections_to_confirm = manager_params.min_detections_to_confirm;

    filter_type_ = stair_manager_params_.filter_type;
    min_detections_to_confirm_ = std::max(1, manager_params.min_detections_to_confirm);

    number_of_stairs_ = 0;

    std::cout << "\033[1;34m[Stair Manager] Initialized Staircase Manager\033[0m" << std::endl;
}

int SingleRobotStairManager::addNewDetectedStaircase(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseEstimate &estimate, bool& is_confirmed)
{

    int id;
    stair_utility::StaircaseProcessingResult result;

    bool found_new = true;
    if (number_of_stairs_ == 0)
    {
        // First incoming stair. Add to list directly.
        id = getNewId();
        
        staircase_database_.emplace(id, std::make_shared<StaircaseModel>(id, new_staircase, filter_type_, stair_manager_params_.filter_sigmas));

        ids_.insert(id);
        number_of_stairs_++;
    }
    else
    {
        // STEP1 : Convert Measurement to Stair Spatial Parameters for correct checking.
        stair_utility::StaircaseInfo stair_info;
        computeStaircaseInfo(new_staircase, stair_info);

        for (auto it = staircase_database_.begin(); it != staircase_database_.end(); it++){
            int score = checkStaircaseSimilarity(it->second->staircase_info_, stair_info); 
            
            if(score >= 1){
                found_new = false;
                id = it->first;
                result = it->second->updateStaircase(new_staircase);
                break;
            }
        }
        if (found_new)
        {
            id = getNewId();
            staircase_database_.emplace(id, std::make_shared<StaircaseModel>(id, new_staircase, filter_type_, stair_manager_params_.filter_sigmas));

            ids_.insert(id);
            number_of_stairs_++;
        }
    }

    
    estimate.stair_count = staircase_database_[id]->stair_count_;
    estimate.stair_id =  id;
    estimate.steps.clear();
    for(int k = 0; k < estimate.stair_count; ++k){
        
        stair_utility::StairStep step; // Step width not used for estimates
        
        step.start_p = staircase_database_[id]->stair_aux_state_.segment((6 * k), 3);
        step.end_p = staircase_database_[id]->stair_aux_state_.segment((6 * k) + 3, 3);

        if(filter_type_ != stair_utility::StaircaseFilterType::SimpleAveraging && filter_type_ != stair_utility::StaircaseFilterType::SimpleMaximum){
            step.line_polar_form = staircase_database_[id]->stair_state_.segment(4 * k, 2);
            step.step_covariance = staircase_database_[id]->stair_covariance_.block(4 * k, 4 * k, 4, 4);
        }
        estimate.steps.push_back(step);
    }
    
    if(!found_new && result.success){
        time_results_.success = true;
        time_results_.data_association_time = result.data_association_time;
        time_results_.filter_time = result.filter_time;
        time_results_.misc_time = result.misc_time;
        time_results_.model_prediction_time =result.model_prediction_time;
    }
    else{
        time_results_.success = false;
        time_results_.data_association_time = 0;
        time_results_.filter_time = 0;
        time_results_.misc_time = 0;
        time_results_.model_prediction_time = 0;
    }
    estimate.staircase_parameters = staircase_database_[id]->staircase_params_;

    // Confirmation gate: only treat the staircase as a real, publishable detection once it has been
    // observed/associated enough times in the world frame. A viewpoint-dependent false positive
    // typically appears once (or lands at an inconsistent world position that never re-associates),
    // so it never reaches the confirmation threshold.
    is_confirmed = (staircase_database_[id]->times_observed_ >= min_detections_to_confirm_);

    std::cout << "\033[1;34m[Stair Manager] Processed Incoming data. Staircase " << id << " has " << staircase_database_[id]->stair_count_ <<" steps, observed " << staircase_database_[id]->times_observed_ << "/" << min_detections_to_confirm_ << " time(s)" << (is_confirmed ? " [confirmed]" : " [pending]") << ". Total staircases in database = " << number_of_stairs_ << " \033[0m" << std::endl;

    return id;
}
