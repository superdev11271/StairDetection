#include "staircase_perception/core/staircase_model.hpp"

StaircaseModel::StaircaseModel(int stair_id, const stair_utility::StaircaseMeasurement& stair_measurement, stair_utility::StaircaseFilterType filter_type, const stair_utility::constantWidthEKFParams& filter_params){

    stair_id_ = stair_id;
    filter_type_ = filter_type;

    stair_count_ = stair_measurement.stair_count;
    stair_state_.resize(stair_measurement.stair_count * 4);
    stair_aux_state_.resize(stair_measurement.stair_count * 6);

    stair_covariance_.setZero(stair_measurement.stair_count * 4, stair_measurement.stair_count * 4);

    Eigen::Matrix4d pose_jacobian = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d measurement_jacobian = Eigen::Matrix4d::Zero();
    
    if(filter_type_ != stair_utility::StaircaseFilterType::SimpleAveraging && filter_type_ != stair_utility::StaircaseFilterType::SimpleMaximum){

        init_measurement_sigmas_ = filter_params.initial_measurement_sigmas.array().square(); 
        init_pose_sigmas_ = filter_params.initial_pose_sigmas.array().square();  // tx, ty, tz, yaw
 
        measurement_sigmas_ = filter_params.measurement_sigmas.array().square();
        pose_sigmas_ = filter_params.pose_sigmas.array().square();  // tx, ty, tz, yaw

        staircase_model_sigmas_ = filter_params.staircase_model_sigmas.array().square();
    }
    
    double tx, ty, tz, r_theta;
    tx = stair_measurement.robot_pose.vehicle_pos.x();
    ty = stair_measurement.robot_pose.vehicle_pos.y();
    tz = stair_measurement.robot_pose.vehicle_pos.z();
    r_theta = stair_measurement.robot_pose.vehicle_quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];
    Eigen::Affine3d transform = stair_measurement.robot_pose.vehicle_pos * stair_measurement.robot_pose.vehicle_quat;

    for(int i = 0; i < stair_count_; i++){

        // Initialize auxillary state vector for endpoints
        stair_aux_state_.segment(6 * i, 3) = transform * stair_measurement.steps[i].start_p;
        stair_aux_state_.segment(6 * i + 3, 3) = transform * stair_measurement.steps[i].end_p;
        
        double d = stair_measurement.steps[i].line_polar_form(0);
        double phi = stair_measurement.steps[i].line_polar_form(1);
        int sign = 1;
        double new_d = d + tx * cos(phi + r_theta) + ty * sin(phi + r_theta);
        // Initialize state vector for line
        if(new_d > 0){
            stair_state_(4 * i) = new_d;
            stair_state_(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta);
            sign = 1;
        }
        else{
            stair_state_(4 * i) = -new_d;
            stair_state_(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta + M_PI);
            sign = -1;
        }
        
        stair_state_(4 * i + 2) = stair_aux_state_(6*i + 2);
        stair_state_(4 * i + 3) = stair_aux_state_(6*i + 5);

        if(filter_type_ != stair_utility::StaircaseFilterType::SimpleAveraging && filter_type_ != stair_utility::StaircaseFilterType::SimpleMaximum){

            measurement_jacobian.setIdentity();
            pose_jacobian.setZero();
 
            measurement_jacobian(0, 1) = sign * (ty * cos(phi + r_theta) - tx * sin(phi + r_theta)); 

            pose_jacobian.row(0) << cos(stair_state_(4 * i + 1)), sin(stair_state_(4 * i + 1)), 0, 0;
            pose_jacobian(1, 3) = 1;
            pose_jacobian(2, 2) = 1;
            pose_jacobian(3, 2) = 1; 
            
            // Initialize covariance 
            if(filter_type_ == stair_utility::StaircaseFilterType::LocalFrameEKF)
                stair_covariance_.block(4*i, 4*i, 4, 4) = pose_jacobian * init_pose_sigmas_.asDiagonal() * pose_jacobian.transpose() + measurement_jacobian * init_measurement_sigmas_.asDiagonal() * measurement_jacobian.transpose();
            else
                stair_covariance_.block(4*i, 4*i, 4, 4) = pose_jacobian * init_pose_sigmas_.asDiagonal() * pose_jacobian.transpose() + measurement_jacobian * stair_measurement.steps[i].step_covariance * measurement_jacobian.transpose();
        }

    }

    computeStaircaseParameters();
    // std::cout << staircase_params_ << std::endl;
    // std::cout << staircase_param_covariance_ << std::endl;

}

void StaircaseModel::computeStaircaseParameters(){
    
    // Compute the parameters of the staircase
    double stair_depth = 0, stair_height = 0, stair_width = 0, direction_delta = 0, prev_direction = 0, curr_step_dir = 0;
    step_directions_.clear();

    for(int i = 0; i < stair_count_; i++){
        if(i == 0){
            // Update Stair Start Direction
            Eigen::Vector3d c_0 = (stair_aux_state_.segment(6*i, 3) + stair_aux_state_.segment(6*i + 3, 3)) / 2;
            Eigen::Vector3d c_1 = (stair_aux_state_.segment(6*(i + 1), 3) + stair_aux_state_.segment(6*(i + 1) + 3, 3)) / 2;

            double dir = atan2(c_1(1) - c_0(1), c_1(0) - c_0(0));
            curr_step_dir = stair_utility::wrap2PI(atan2(stair_aux_state_(6 * i + 4) - stair_aux_state_(6 * i + 1), stair_aux_state_(6 * i + 3) - stair_aux_state_(6 * i)));
            
            if(fabs(stair_utility::wrap2PI(curr_step_dir + M_PI_2 - dir)) < fabs(stair_utility::wrap2PI(curr_step_dir - M_PI_2 - dir))){
                staircase_params_(3) = stair_utility::wrap2PI(curr_step_dir + M_PI_2);
                prev_direction = staircase_params_(3);
                step_directions_.push_back(staircase_params_(3));
            }
            else{
                staircase_params_(3) = stair_utility::wrap2PI(curr_step_dir - M_PI_2);
                prev_direction = staircase_params_(3);
                step_directions_.push_back(staircase_params_(3));
            }
                
            // Stair Height Computation
            stair_height += (fabs(stair_aux_state_(6*(i + 1) + 2) - stair_aux_state_(6*i + 2)) + fabs(stair_aux_state_(6*(i + 1) + 5) - stair_aux_state_(6*i + 5)));

            // Stair Depth Compuation
            stair_depth += stair_utility::get_line_distance(stair_aux_state_.segment(6 * (i + 1), 3), stair_aux_state_.segment(6 * (i + 1) + 3, 3), c_0);

        }
        else if(i == stair_count_ - 1){
            // Update Stair End Direction
            Eigen::Vector2d c_1 = (stair_aux_state_.segment(6*i, 2) + stair_aux_state_.segment(6*i + 3, 2)) / 2;
            Eigen::Vector2d c_0 = (stair_aux_state_.segment(6*(i - 1), 2) + stair_aux_state_.segment(6*(i - 1) + 3, 2)) / 2;
            
            double dir = atan2(c_1(1) - c_0(1), c_1(0) - c_0(0));
            curr_step_dir = stair_utility::wrap2PI(atan2(stair_aux_state_(6 * i + 4) - stair_aux_state_(6 * i + 1), stair_aux_state_(6 * i + 3) - stair_aux_state_(6 * i)));

            if(fabs(stair_utility::wrap2PI(curr_step_dir + M_PI_2 - dir)) < fabs(stair_utility::wrap2PI(curr_step_dir - M_PI_2 - dir))){
                staircase_params_(5) = stair_utility::wrap2PI(curr_step_dir + M_PI_2);
                step_directions_.push_back(staircase_params_(5));
                
                direction_delta += stair_utility::wrap2PI(staircase_params_(5) - prev_direction);
                prev_direction = staircase_params_(5);
            }
            else{
                staircase_params_(5) = stair_utility::wrap2PI(curr_step_dir - M_PI_2);
                step_directions_.push_back(staircase_params_(5));

                direction_delta += stair_utility::wrap2PI(staircase_params_(5) - prev_direction);
                prev_direction = staircase_params_(5);
            }

        }
        else{ 
            // Stair Height Computation
            stair_height += (fabs(stair_aux_state_(6*i + 2) - stair_aux_state_(6*(i + 1) + 2)) + fabs(stair_aux_state_(6*i + 5) - stair_aux_state_(6*(i + 1) + 5)));

            // Delta Direction Computation
            Eigen::Vector3d c_0 = (stair_aux_state_.segment(6*i, 3) + stair_aux_state_.segment(6*i + 3, 3)) / 2;
            Eigen::Vector3d c_1 = (stair_aux_state_.segment(6*(i + 1), 3) + stair_aux_state_.segment(6*(i + 1) + 3, 3)) / 2;

            double dir = atan2(c_1(1) - c_0(1), c_1(0) - c_0(0));
            curr_step_dir = stair_utility::wrap2PI(atan2(stair_aux_state_(6 * i + 4) - stair_aux_state_(6 * i + 1), stair_aux_state_(6 * i + 3) - stair_aux_state_(6 * i)));

            if(fabs(stair_utility::wrap2PI(curr_step_dir + M_PI_2 - dir)) < fabs(stair_utility::wrap2PI(curr_step_dir - M_PI_2 - dir))){
                double temp = stair_utility::wrap2PI(curr_step_dir + M_PI_2);
                step_directions_.push_back(temp);

                direction_delta += stair_utility::wrap2PI(temp - prev_direction);
                prev_direction = temp;
            }
            else{
                double temp = stair_utility::wrap2PI(curr_step_dir - M_PI_2);
                step_directions_.push_back(temp);
                
                direction_delta += stair_utility::wrap2PI(temp - prev_direction);
                prev_direction = temp;
            }

            // Stair Depth Compuation
            stair_depth += stair_utility::get_line_distance(stair_aux_state_.segment(6 * (i + 1), 3), stair_aux_state_.segment(6 * (i + 1) + 3, 3), c_0);

        }

        // Stair Width Computation
        double curr_width = (stair_aux_state_.segment(6*i, 3) - stair_aux_state_.segment(6*i + 3, 3)).norm();
        stair_width += curr_width;
        
    }

    staircase_params_(0) = stair_height / (2.0 * (stair_count_ - 1));
    staircase_params_(1) = stair_depth / (stair_count_ - 1);
    staircase_params_(2) = stair_width / stair_count_;
    staircase_params_(4) = direction_delta / (stair_count_ - 1);

    // Compute polygon and general direction and update staircase info variable;
    staircase_info_.stair_polygon.clear();

    // From the two point of the last stair, ensure the polygon is going in order.  
    float theta1 = atan2(stair_aux_state_(6 * (stair_count_ - 1) + 4) - stair_aux_state_(1), stair_aux_state_(6 * (stair_count_ - 1) + 3) - stair_aux_state_(0));
    float theta2 = atan2(stair_aux_state_(6 * (stair_count_ - 1) + 1) - stair_aux_state_(1), stair_aux_state_(6 * (stair_count_ - 1)) - stair_aux_state_(0));
    float theta0 = atan2(stair_aux_state_(4) - stair_aux_state_(1), stair_aux_state_(3) - stair_aux_state_(0));

    if(fabs(stair_utility::wrap2PI(theta1 - theta0)) < fabs(stair_utility::wrap2PI(theta2 - theta0))){
        // Insert the start and end point of the first staircase.
        Eigen::Vector3d offset1 = {1.15 * staircase_params_(1) * cos(theta1), 1.15 * staircase_params_(1) * sin(theta1), 0.12};
        float theta3 = atan2(stair_aux_state_(6 * (stair_count_ - 1) + 1) - stair_aux_state_(4), stair_aux_state_(6 * (stair_count_ - 1)) - stair_aux_state_(3));
        Eigen::Vector3d offset2 = {1.15 * staircase_params_(1) * cos(theta3), 1.15 * staircase_params_(1) * sin(theta3), 0.12};

        // Eigen::Vector3d offset = {staircase_params_(1) * cos(step_directions_[stair_count_ - 1]), staircase_params_(1) * sin(step_directions_[stair_count_ - 1]) ,staircase_params_(0)};
        
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(0, 3) - offset1);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(3, 3) - offset2);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(6 * (stair_count_ - 1) + 3, 3) + offset1);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(6 * (stair_count_ - 1), 3) + offset2);
    }
    else{
        // Insert the start and end point of the first staircase.
        Eigen::Vector3d offset1 = {1.15 * staircase_params_(1) * cos(theta2), 1.15 * staircase_params_(1) * sin(theta2), 0.12};
        float theta3 = atan2(stair_aux_state_(6 * (stair_count_ - 1) + 4) - stair_aux_state_(4), stair_aux_state_(6 * (stair_count_ - 1) + 3) - stair_aux_state_(3));
        Eigen::Vector3d offset2 = {1.15 * staircase_params_(1) * cos(theta3), 1.15 * staircase_params_(1) * sin(theta3), 0.12};

        // Eigen::Vector3d offset = {staircase_params_(1) * cos(step_directions_[stair_count_ - 1]), staircase_params_(1) * sin(step_directions_[stair_count_ - 1]) ,staircase_params_(0)};
        
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(0, 3) - offset1);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(3, 3) - offset2);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(6 * (stair_count_ - 1), 3) + offset1);
        staircase_info_.stair_polygon.push_back(stair_aux_state_.segment(6 * (stair_count_ - 1) + 3, 3) + offset2);
    }

    Eigen::Vector3d center_s = (stair_aux_state_.segment(0, 3) + stair_aux_state_.segment(3, 3))/2;
    Eigen::Vector3d center_e = (stair_aux_state_.segment(6 * (stair_count_ - 1), 3) + stair_aux_state_.segment(6 * (stair_count_ - 1) + 3, 3))/2;
    float cos_s = 0, sin_s = 0;
    for(int j = 0; j < step_directions_.size(); j++){
        cos_s += cos(step_directions_[j]);
        sin_s += sin(step_directions_[j]);
    }
    staircase_info_.staircase_direction = atan2(sin_s, cos_s);
    
}

bool StaircaseModel::computeMeasurementMatches(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps){
    matched_measurements_.clear();
    preceding_measurements_.clear();
    succeeding_measurements_.clear();
    // unmatched_states_.clear();
    matches_from_current_state_.clear();
    unmatched_measurements_.clear();

    bool result = true; // print_matches = false;
    int highest_preceeding = -1, lowest_succeeding = INT_MAX;

    for(int i = 0; i < incoming_stair_count; i++){
        
        bool matched = false;
        double incoming_z1 = new_staircase_steps[i].start_p(2);
        double incoming_z2 = new_staircase_steps[i].end_p(2);
        double incoming_z = (incoming_z1 + incoming_z2)/2;
        
        for(int j = 0; j < stair_count_; j++){
            double curr_z1 = stair_aux_state_((6 * j) + 2);
            double curr_z2 = stair_aux_state_((6 * j) + 5);
            double curr_z = (curr_z1 + curr_z2)/2;

            if(fabs(incoming_z - curr_z) <= 0.08){
                if(matched_measurements_.find(i) == matched_measurements_.end()){
                    if(matches_from_current_state_.find(j) == matches_from_current_state_.end()){
                        matched_measurements_.insert({i, j});
                        matches_from_current_state_.insert({j, i});
                    }
                    else{
                        // Something weird happened, check stuff
                        // std::cout << "\033[0;36m[Stair Matching] Conflict in matching, trying to resolve \033[0m" << std::endl;
                        // print_matches = true;
                        std::deque<int> backtrack_queue;

                        backtrack_queue.push_back(j);
                        int curr_measurement_match = i, curr_state_step = j;
                        bool backtrack_active = false;

                        while(!backtrack_queue.empty() && curr_state_step >= 0){
                            curr_state_step = backtrack_queue.front();
                            backtrack_queue.pop_front();

                            int prev_measurement_match = matches_from_current_state_.at(curr_state_step);
                            // std::cout << "\033[0;36m[Stair Matching] " << curr_state_step << " matched with measure " << curr_measurement_match  << " and prev measure " << prev_measurement_match << "\033[0m" << std::endl;

                            Eigen::Vector3d center_prev = (new_staircase_steps[prev_measurement_match].start_p + new_staircase_steps[prev_measurement_match].end_p) / 2;
                            Eigen::Vector3d center_curr = (new_staircase_steps[curr_measurement_match].start_p + new_staircase_steps[curr_measurement_match].end_p) / 2;
                            float dist1 = stair_utility::get_line_distance(stair_aux_state_.segment((6 * curr_state_step), 3), stair_aux_state_.segment((6 * curr_state_step) + 3, 3), center_prev);  // Dist between previous matched measurement and matched state
                            float dist2 = stair_utility::get_line_distance(stair_aux_state_.segment((6 * curr_state_step), 3), stair_aux_state_.segment((6 * curr_state_step) + 3, 3), center_curr);   // Dist between current matched measurement and matched state
                            
                            if(dist1 < dist2 && !backtrack_active){
                                // curr match needs to j+1
                                if(j + 1 < stair_count_){
                                    matched_measurements_.insert({i, j + 1});
                                    matches_from_current_state_.insert({j + 1, i});
                                }
                                else{
                                    // std::cout << "[Stair Matching] Prev Match is more correct and current match is classified as succeeding!" << std::endl;
                                    if(i < lowest_succeeding)
                                        lowest_succeeding = i;
                                    succeeding_measurements_.insert(i);
                                }
                                
                            }   
                            else{
                                // Back prop logic to adjust stuff, for start, just back-prop once assuming best case scenarios
                                backtrack_active = true;

                                matched_measurements_[curr_measurement_match] = curr_state_step;
                                if(curr_state_step - 1 >= 0){
                                    matched_measurements_[prev_measurement_match] = curr_state_step - 1;
                                    matches_from_current_state_[curr_state_step] = curr_measurement_match;

                                    if(matches_from_current_state_.find(curr_state_step - 1) != matches_from_current_state_.end()){
                                        // std::cout << "\033[0;36m[Stair Matching] Continuing to backtrack\033[0m" << std::endl;
                                        backtrack_queue.push_back(curr_state_step - 1);
                                        curr_measurement_match = prev_measurement_match;
                                    }
                                    else{
                                        matches_from_current_state_.insert({curr_state_step - 1, prev_measurement_match});
                                    }
                                }
                                else{
                                    // Backtracked to the inital step, and added to preceeding
                                    matched_measurements_.erase(prev_measurement_match);
                                    preceding_measurements_.insert(prev_measurement_match);
                                    if(prev_measurement_match > highest_preceeding)
                                        highest_preceeding = prev_measurement_match;
                                }

                            }
                        }

                    }

                    matched = true;
                }
                else{
                    std::cout << "\033[0;36m[Stair Matching] Something incorrect, double match search occuring\033[0m" << std::endl; 
                    matched = false;
                }
                
                break;
            }

        }

        if(!matched){
            // Check if preceeding or succeeding
            float z_0 = (stair_aux_state_(2) + stair_aux_state_(5))/2;
            if(incoming_z < (z_0 - 0.08)){
                if(i > highest_preceeding)
                    highest_preceeding = i;
                preceding_measurements_.insert(i);
                matched = true;
                
            }
            float z_n = (stair_aux_state_(6*(stair_count_ - 1) + 2) + stair_aux_state_(6*(stair_count_ - 1) + 5))/2;
            if(incoming_z > (z_n + 0.08)){
                if(i < lowest_succeeding)
                    lowest_succeeding = i;
                succeeding_measurements_.insert(i);
                matched = true;
            }
        }

        if(!matched && i > 0){
            int prev_match = matched_measurements_[i - 1];
            // std::cout << "prev-match : " << prev_match << " i: " << i << "i - 1" << i -1 << std::endl;
            if(matches_from_current_state_.find(prev_match + 1) == matches_from_current_state_.end()){
                matched_measurements_.insert({i, prev_match + 1});
                matches_from_current_state_.insert({prev_match + 1, i});
                
                // print_matches = true;
                matched = true;
            }
        }

        // Ensure preceeding measurements are not skipping anything
        if(matched){
            if(preceding_measurements_.size() > 0){
                if(matches_from_current_state_.size() > 0){
                    if(matches_from_current_state_.find(0) == matches_from_current_state_.end()){
                        std::cout << "\033[0;36m[Stair Matching] Incorrect first step matched as preceeding, adjusting\033[0m" << std::endl;
                        preceding_measurements_.erase(highest_preceeding);
                        matched_measurements_.insert({highest_preceeding, 0});
                        matches_from_current_state_.insert({0, highest_preceeding});
                        // print_matches = true;
                    }
                }
            }

            if(succeeding_measurements_.size() > 0){
                if(matches_from_current_state_.size() > 0){
                    if(matches_from_current_state_.find(stair_count_ - 1) == matches_from_current_state_.end()){
                        std::cout << "\033[0;36m[Stair Matching] Incorrect last step matched as succeeding, adjusting\033[0m" << std::endl;
                        succeeding_measurements_.erase(lowest_succeeding);
                        matched_measurements_.insert({lowest_succeeding, stair_count_ - 1});
                        matches_from_current_state_.insert({stair_count_ - 1, lowest_succeeding});
                        // print_matches = true;
                    }
                }
            }
        }

        result = result && matched;  
        
        if(!matched){
            unmatched_measurements_.insert(i);
        }
    }

    // std::cout << "Matches: " << std::endl;
    // for(auto it = matched_measurements_.begin(); it != matched_measurements_.end(); ++it){
    //     std::cout << "M: " << it->first << " S: " << it->second << std::endl;
    // }
    // std::cout << "Inverse: " << std::endl;
    // for(auto it = matches_from_current_state_.begin(); it != matches_from_current_state_.end(); ++it){
    //     std::cout << "M: " << it->second << " S: " << it->first << std::endl;
    // }

    // Verify Sanity of Matches - Returns true is matching is valid and can be applied. Returns false
    if(!result){
        if((*unmatched_measurements_.rbegin() > matched_measurements_.begin()->first) && (*unmatched_measurements_.begin() < matched_measurements_.rbegin()->first)){
            // Maybe consider rejecting measurement due to outliers?!
            std::cout << "\033[0;36m[Stair Matching] No match found for some stairs, detection too noisy compared to previous ones! Throwing away measurements\033[0m" << std::endl;
            return false;
        }
        else{
            // Check sanity of matches!
            if(matches_from_current_state_.size() != matched_measurements_.size()){
                std::cout << "\033[0;36m[Stair Matching] Throwing away measurement due to improper matching!\033[0m" << std::endl;
                return false;
            };

            if(matches_from_current_state_.size() > 0){
                int state_matches_actual = (matches_from_current_state_.rbegin()->first - matches_from_current_state_.begin()->first) + 1;
                if(state_matches_actual != matches_from_current_state_.size()){
                    std::cout << "\033[0;36m[Stair Matching] Error in Matching due to non-continous state matching! Throwing away measurement\033[0m" << std::endl;
                    return false;
                }
            }

            std::cout << "\033[0;36m[Stair Matching] No match found for some measurements, but measurement is at the edges, ignoring some measurements, but still merging!\033[0m" << std::endl;
            std::cout << "\033[1;36m[Stair Matching] Matches: " << matched_measurements_.size() << " Preceeding: " << preceding_measurements_.size() << " Suceeding: " << succeeding_measurements_.size() << "\033[0m" << std::endl;
        }   
    }
    else{
        // Check sanity of matches!
        if(matches_from_current_state_.size() != matched_measurements_.size()){
            std::cout << "\033[0;36m[Stair Matching] Throwing away measurement due to improper matching!\033[0m" << std::endl;
            return false;
        };
            
        if(matches_from_current_state_.size() > 0){
            int state_matches_actual = (matches_from_current_state_.rbegin()->first - matches_from_current_state_.begin()->first) + 1;
            if(state_matches_actual != matches_from_current_state_.size()){
                std::cout << "\033[0;36m[Stair Matching] Error in Matching due to non-continous state matching! Throwing away measuremement \033[0m" << std::endl;
                return false;
            }
        }

        std::cout << "\033[1;36m[Stair Matching] Matches: " << matched_measurements_.size() << " Preceeding: " << preceding_measurements_.size() << " Suceeding: " << succeeding_measurements_.size() << "\033[0m" << std::endl;
    }

    return true;
    
}

bool StaircaseModel::computeMeasurementMatchesWithCovarianceInLocalFrame(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps, const Eigen::Affine3d& transform){
    matched_measurements_.clear();
    preceding_measurements_.clear();
    succeeding_measurements_.clear();
    matches_from_current_state_.clear();
    unmatched_measurements_.clear();

    Eigen::VectorXd local_stair_state(stair_count_ * 4);
    Eigen::MatrixXd local_stair_variance;
    Eigen::MatrixXd state_jacobian(stair_count_ * 4, stair_count_ * 4);
    state_jacobian.setIdentity();
    Eigen::MatrixXd pose_jacobian(stair_count_ * 4, stair_count_ * 4);
    pose_jacobian.setZero();

    Eigen::Affine3d inverse_transform = transform.inverse();
    
    double tx, ty, tz, r_theta;
    r_theta = inverse_transform.rotation().eulerAngles(0, 1, 2)[2];
    tx = inverse_transform.translation().x();
    ty = inverse_transform.translation().y();
    tz = inverse_transform.translation().z();

    for(int i = 0; i < stair_count_; i++){
        
        double d = stair_state_(4 * i);
        double phi = stair_state_(4 * i + 1);

        // Compute Expected Local State
        double new_d = d + tx * cos(phi + r_theta) + ty * sin(phi + r_theta);
        if(new_d < 0){
            local_stair_state(4 * i) = -new_d;
            local_stair_state(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta + M_PI);
             // Compute Jacobian wrt to state terms

            state_jacobian(4 * i, 4 * i + 1) =  - (ty * cos(phi + r_theta) - tx * sin(phi + r_theta)); 
        
            // Compute Jacobian wrt to pose terms
            pose_jacobian(4 * i, 4 * i) = cos(local_stair_state(4 * i + 1));
            pose_jacobian(4 * i, 4 * i + 1) =  sin(local_stair_state(4 * i + 1));
            pose_jacobian(4 * i + 1, 4 * i + 3) = 1;
            pose_jacobian(4 * i + 2, 4 * i + 2) = 1;
            pose_jacobian(4 * i + 3, 4 * i + 2) = 1; 
            // std::cout << "HERE IN local state R NEGATIVE" << std::endl;
        }
        else{
            local_stair_state(4 * i) = new_d;
            local_stair_state(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta);

             // Compute Jacobian wrt to state terms
            state_jacobian(4 * i, 4 * i + 1) =  ty * cos(phi + r_theta) - tx * sin(phi + r_theta); 
        
            // Compute Jacobian wrt to pose terms
            pose_jacobian(4 * i, 4 * i) = cos(phi + r_theta);
            pose_jacobian(4 * i, 4 * i + 1) = sin(phi + r_theta);
            pose_jacobian(4 * i + 1, 4 * i + 3) = 1;
            pose_jacobian(4 * i + 2, 4 * i + 2) = 1;
            pose_jacobian(4 * i + 3, 4 * i + 2) = 1; 
        }

        local_stair_state(4 * i + 2) = stair_state_(4 * i + 2) + tz;
        local_stair_state(4 * i + 3) = stair_state_(4 * i + 3) + tz;    

    }
    Eigen::MatrixXd pose_sigma_block = pose_sigmas_.replicate(stair_count_, 1).asDiagonal();
    local_stair_variance = (state_jacobian * stair_covariance_ * state_jacobian.transpose()) + (pose_jacobian * pose_sigma_block * pose_jacobian.transpose());

    bool result = true;
    int highest_preceeding = -1, lowest_succeeding = INT_MAX;

    // Store potential matches with their Mahalanobis distances
    std::unordered_map<int, std::unordered_map<int, double>> potential_matches;
    std::unordered_map<int, double> min_mahalanobis_dist;

    for(int i = 0; i < incoming_stair_count; i++){
        double incoming_z1 = new_staircase_steps[i].start_p(2);
        double incoming_z2 = new_staircase_steps[i].end_p(2);
        double incoming_z = (incoming_z1 + incoming_z2)/2;
        potential_matches[i] = {};
       
        for(int j = 0; j < stair_count_; j++){
            double curr_z1 = local_stair_state((4 * j) + 2);
            double curr_z2 = local_stair_state((4 * j) + 3);
            double sigma_z1 = local_stair_variance((4 * j) + 2, (4 * j) + 2);
            double sigma_z2 = local_stair_variance((4 * j) + 3, (4 * j) + 3);

            double chi_2 =  (std::pow((curr_z1 - incoming_z1), 2)/sigma_z1 + std::pow((curr_z2 - incoming_z2), 2)/sigma_z2);
            
            if(chi_2 <= 3.1){
              Eigen::Matrix2d d_phi_covar = local_stair_variance.block(4 * j, 4 * j, 2, 2);
              Eigen::Vector2d del_L = new_staircase_steps[i].line_polar_form - local_stair_state.segment(4 * j, 2);
              double mahalanobis_dist = sqrt(del_L.transpose() * d_phi_covar.inverse() * del_L);
              potential_matches[i][j] = chi_2;
            }
        }
    }
    
    //Assign matches based on minimum mahalanobis distance
    for (auto & [meas_idx, state_matches] : potential_matches){
          double min_dist = std::numeric_limits<double>::max();
          int best_state_idx = -1;

            for (const auto& [state_idx, dist] : state_matches){
              if (dist < min_dist){
                bool matched_already = false;
                 for (const auto& [matched_state, matched_meas] : matches_from_current_state_)
                {
                     if(matched_state == state_idx)
                        matched_already = true;
                }
                if (!matched_already){
                  min_dist = dist;
                  best_state_idx = state_idx;
                }
              }
            }
            
            if (best_state_idx != -1){
              matched_measurements_.insert({meas_idx, best_state_idx});
              matches_from_current_state_.insert({best_state_idx, meas_idx});
            }
            else{
              unmatched_measurements_.insert(meas_idx);
            }
    }

    // Handle unassigned measurements based on z-height
    for(int i = 0; i < incoming_stair_count; i++){
      if(matched_measurements_.find(i) == matched_measurements_.end()){
            
            double incoming_z1 = new_staircase_steps[i].start_p(2);
            double incoming_z2 = new_staircase_steps[i].end_p(2);
            double incoming_z = (incoming_z1 + incoming_z2)/2;
            float z_0 = (local_stair_state(2) + local_stair_state(3))/2;
            if(incoming_z < (z_0 - 0.06)){
                if(i > highest_preceeding)
                    highest_preceeding = i;
                preceding_measurements_.insert(i);
                
            }
            float z_n = (local_stair_state(4*(stair_count_ - 1) + 2) + local_stair_state(4*(stair_count_ - 1) + 3))/2;
            if(incoming_z > (z_n + 0.06)){
                if(i < lowest_succeeding)
                    lowest_succeeding = i;
                succeeding_measurements_.insert(i);
            }
        }
    }

    // Fill in gaps between matched steps
    for(int i = 0; i < incoming_stair_count; i++){
        if(matched_measurements_.find(i) == matched_measurements_.end() && i > 0){
           int prev_match = -1;
            if(matched_measurements_.find(i-1) != matched_measurements_.end())
              prev_match = matched_measurements_[i - 1];
            if(prev_match != -1){
              if(matches_from_current_state_.find(prev_match + 1) == matches_from_current_state_.end()){
                  matched_measurements_.insert({i, prev_match + 1});
                  matches_from_current_state_.insert({prev_match + 1, i});

              }
            }
        }
    }

    // Ensure preceeding measurements are not skipping anything
     if(preceding_measurements_.size() > 0){
        if(matches_from_current_state_.size() > 0){
            if(matches_from_current_state_.find(0) == matches_from_current_state_.end()){
                std::cout << "\033[0;36m[Stair Matching] Incorrect first step matched as preceeding, adjusting\033[0m" << std::endl;
                preceding_measurements_.erase(highest_preceeding);
                for(auto it = potential_matches[highest_preceeding].begin(); it != potential_matches[highest_preceeding].end(); ++it){
                  if (it->first == 0){
                     matched_measurements_.insert({highest_preceeding, 0});
                     matches_from_current_state_.insert({0, highest_preceeding});
                  }
                }
            }
        }
    }

    if(succeeding_measurements_.size() > 0){
        if(matches_from_current_state_.size() > 0){
            if(matches_from_current_state_.find(stair_count_ - 1) == matches_from_current_state_.end()){
                std::cout << "\033[0;36m[Stair Matching] Incorrect last step matched as succeeding, adjusting\033[0m" << std::endl;
                succeeding_measurements_.erase(lowest_succeeding);
                for(auto it = potential_matches[lowest_succeeding].begin(); it != potential_matches[lowest_succeeding].end(); ++it){
                  if (it->first == stair_count_ - 1){
                     matched_measurements_.insert({lowest_succeeding, stair_count_ - 1});
                     matches_from_current_state_.insert({stair_count_ - 1, lowest_succeeding});
                  }
                }
            }
        }
    }

    //check if something is unmatached
    for (int un_i = 0; un_i < incoming_stair_count; un_i++){
        if(matched_measurements_.find(un_i) == matched_measurements_.end()){
            unmatched_measurements_.insert(un_i);
        }
    }

    // Verify Sanity of Matches - Returns true is matching is valid and can be applied. Returns false
   if(matches_from_current_state_.size() != matched_measurements_.size()){
        std::cout << "\033[0;36m[Stair Matching] Throwing away measurement due to improper matching!\033[0m" << std::endl;
        return false;
    };
        
    if(matches_from_current_state_.size() > 0){
        int state_matches_actual = (matches_from_current_state_.rbegin()->first - matches_from_current_state_.begin()->first) + 1;
        if(state_matches_actual != matches_from_current_state_.size()){
            std::cout << "\033[0;36m[Stair Matching] Error in Matching due to non-continous state matching! Throwing away measuremement \033[0m" << std::endl;
            return false;
        }
    }

    if(unmatched_measurements_.size() > 0 && matched_measurements_.size() != 0){
        if((*unmatched_measurements_.rbegin() > matched_measurements_.begin()->first) && (*unmatched_measurements_.begin() < matched_measurements_.rbegin()->first)){
            // Maybe consider rejecting measurement due to outliers?!
            std::cout << "\033[0;36m[Stair Matching] No match found for some stairs, detection too noisy compared to previous ones! Throwing away measurements\033[0m" << std::endl;
            return false;
        }
        else{
           std::cout << "\033[0;36m[Stair Matching] No match found for some measurements, but measurement is at the edges, ignoring some measurements, but still merging!\033[0m" << std::endl;
           std::cout << "\033[1;36m[Stair Matching] Matches: " << matched_measurements_.size() << " Preceeding: " << preceding_measurements_.size() << " Suceeding: " << succeeding_measurements_.size() << "\033[0m" << std::endl;
        }
    }
    else{
        std::cout << "\033[1;36m[Stair Matching] Matches: " << matched_measurements_.size() << " Preceeding: " << preceding_measurements_.size() << " Suceeding: " << succeeding_measurements_.size() << "\033[0m" << std::endl;
    }

    return true;    
}

void StaircaseModel::applyAveragingFilter(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps){
    // std::cout << "\033[1;36m[Stair Merging] Averaging matchces now\033[0m" << std::endl;
    for (auto it = matched_measurements_.begin(); it != matched_measurements_.end(); it++){
        int measuremementIdx = it->first, stateIdx = it->second;
        
        float state_line_dir = atan2(stair_aux_state_((6 * stateIdx) + 4) - stair_aux_state_((6 * stateIdx) + 1), stair_aux_state_((6 * stateIdx) + 3) - stair_aux_state_((6 * stateIdx)));
        float measure_line_dir = atan2(new_staircase_steps[measuremementIdx].end_p(1) - new_staircase_steps[measuremementIdx].start_p(1), new_staircase_steps[measuremementIdx].end_p(0) - new_staircase_steps[measuremementIdx].start_p(0));

        Eigen::Vector3d new_start, new_end;
        if (fabs(stair_utility::wrap2PI(state_line_dir - measure_line_dir)) < M_PI_2) {
            // Average Starts and Ends Together
            new_start = (stair_aux_state_.segment((6 * stateIdx), 3) + new_staircase_steps[measuremementIdx].start_p) / 2;
            new_end = (stair_aux_state_.segment((6 * stateIdx) + 3, 3) +  new_staircase_steps[measuremementIdx].end_p) / 2;
        } else {
            // Average Starts with Ends (reverse the measurement points)
            new_start = (stair_aux_state_.segment((6 * stateIdx), 3) + new_staircase_steps[measuremementIdx].end_p) / 2;
            new_end = (stair_aux_state_.segment((6 * stateIdx) + 3, 3) +  new_staircase_steps[measuremementIdx].start_p) / 2;
        }
        stair_aux_state_.segment((6 * stateIdx), 3) = new_start;
        stair_aux_state_.segment((6 * stateIdx) + 3, 3) = new_end;
    }

    // std::cout << "Averaged" << std::endl;
    // std::cout << stair_aux_state_ << std::endl;
    
    // For unmatched stairs, prepend or append things - Resize vector and add to new state;
    // std::cout << "\033[1;36m[Stair Merging] Adding unmatchced now\033[0m" << std::endl;
    int new_stair_count = stair_count_ + (preceding_measurements_.size() + succeeding_measurements_.size()); // 6 values per step

    if(new_stair_count != stair_count_){

        Eigen::VectorXd new_stair_aux_state_(new_stair_count * 6);

        int offset = 6 * (preceding_measurements_.size() - 1);
        for (auto it = preceding_measurements_.rbegin(); it != preceding_measurements_.rend(); it++){
            int stepIdx = *it; 
            new_stair_aux_state_.segment(offset, 3) = new_staircase_steps[stepIdx].start_p;
            new_stair_aux_state_.segment(offset + 3, 3) = new_staircase_steps[stepIdx].end_p;

            offset -= 6;
        }

        // std::cout << "Preceeded" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;

        new_stair_aux_state_.segment(6 * preceding_measurements_.size(), 6 * stair_count_) = stair_aux_state_;

        // std::cout << "Old Data in" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;
        offset = (6 * new_stair_count) - (6 * succeeding_measurements_.size());
        for (auto it = succeeding_measurements_.begin(); it != succeeding_measurements_.end(); it++){
            int stepIdx = *it; 
            new_stair_aux_state_.segment(offset, 3) = new_staircase_steps[stepIdx].start_p;
            new_stair_aux_state_.segment(offset + 3, 3) = new_staircase_steps[stepIdx].end_p;

            offset += 6;
        }

        stair_count_ = new_stair_count;
        // std::cout << "Succeeded" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;
        
        stair_aux_state_ = std::move(new_stair_aux_state_);
    }
    // std::cout << "Final" << std::endl;
    // std::cout << stair_aux_state_ << std::endl;

}

void StaircaseModel::applyMaximizingFilter(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps){
    // std::cout << "\033[1;36m[Stair Merging] Averaging matchces now\033[0m" << std::endl;
    for (auto it = matched_measurements_.begin(); it != matched_measurements_.end(); it++){
        int measuremementIdx = it->first, stateIdx = it->second;
        
        float state_line_dir = atan2(stair_aux_state_((6 * stateIdx) + 4) - stair_aux_state_((6 * stateIdx) + 1), stair_aux_state_((6 * stateIdx) + 3) - stair_aux_state_((6 * stateIdx)));


        std::pair<Eigen::Vector3d, Eigen::Vector3d> longestSegment;
        std::vector<Eigen::Vector3d> points = {stair_aux_state_.segment((6 * stateIdx), 3), stair_aux_state_.segment((6 * stateIdx) + 3, 3), new_staircase_steps[measuremementIdx].start_p, new_staircase_steps[measuremementIdx].end_p}; 
        double maxDistanceSquared = 0.0;
        for (size_t loop1 = 0; loop1 < points.size(); ++loop1) {
            for (size_t loop2 = loop1 + 1; loop2 < points.size(); ++loop2) {
                double distSq = (points[loop1].segment(0, 2) - points[loop2].segment(0, 2)).squaredNorm();

                if (distSq > maxDistanceSquared) {
                    maxDistanceSquared = distSq;
                    longestSegment = std::make_pair(points[loop1], points[loop2]);
                }
            }
        }

        double new_line_dir = atan2(longestSegment.second.y() - longestSegment.first.y(), longestSegment.second.x() - longestSegment.first.x()); 
        if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
            stair_aux_state_.segment((6 * stateIdx), 3) = longestSegment.first;
            stair_aux_state_.segment((6 * stateIdx) + 3, 3) = longestSegment.second;
        }
        else{
            stair_aux_state_.segment((6 * stateIdx), 3) = longestSegment.second;
            stair_aux_state_.segment((6 * stateIdx) + 3, 3) = longestSegment.first;
        }
    }

    // std::cout << "Maiximized" << std::endl;
    // std::cout << stair_aux_state_ << std::endl;
    
    // For unmatched stairs, prepend or append things - Resize vector and add to new state;
    // std::cout << "\033[1;36m[Stair Merging] Adding unmatchced now\033[0m" << std::endl;
    int new_stair_count = stair_count_ + (preceding_measurements_.size() + succeeding_measurements_.size()); // 6 values per step

    if(new_stair_count != stair_count_){

        Eigen::VectorXd new_stair_aux_state_(new_stair_count * 6);

        int offset = 6 * (preceding_measurements_.size() - 1);
        for (auto it = preceding_measurements_.rbegin(); it != preceding_measurements_.rend(); it++){
            int stepIdx = *it; 
            new_stair_aux_state_.segment(offset, 3) = new_staircase_steps[stepIdx].start_p;
            new_stair_aux_state_.segment(offset + 3, 3) = new_staircase_steps[stepIdx].end_p;

            offset -= 6;
        }

        // std::cout << "Preceeded" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;

        new_stair_aux_state_.segment(6 * preceding_measurements_.size(), 6 * stair_count_) = stair_aux_state_;

        // std::cout << "Old Data in" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;
        offset = (6 * new_stair_count) - (6 * succeeding_measurements_.size());
        for (auto it = succeeding_measurements_.begin(); it != succeeding_measurements_.end(); it++){
            int stepIdx = *it; 
            new_stair_aux_state_.segment(offset, 3) = new_staircase_steps[stepIdx].start_p;
            new_stair_aux_state_.segment(offset + 3, 3) = new_staircase_steps[stepIdx].end_p;

            offset += 6;
        }

        stair_count_ = new_stair_count;
        // std::cout << "Succeeded" << std::endl;
        // std::cout << new_stair_aux_state_ << std::endl;
        
        stair_aux_state_ = std::move(new_stair_aux_state_);
    }
    // std::cout << "Final" << std::endl;
    // std::cout << stair_aux_state_ << std::endl;

}

void StaircaseModel::predictStaircaseStateFromModel(const int measurement_stair_count, const std::vector<stair_utility::StairStep> &measurement_steps, const Eigen::Affine3d &transform){
    
    int new_stair_count = preceding_measurements_.size() +  stair_count_ + succeeding_measurements_.size();    
    // int new_stair_count =  stair_count_ ;    

    predicted_state_.resize(new_stair_count * 4);
    predicted_aux_state_.resize(new_stair_count * 6);

    input_measurement_vector_.resize(new_stair_count * 4);
    registered_input_measurement_.resize(new_stair_count * 4);
    registered_input_aux_measurement_.resize(new_stair_count * 6);

    predicted_covariance_.resize(new_stair_count * 4, new_stair_count * 4);
    active_measurements_.setIdentity(new_stair_count * 4, new_stair_count * 4);
    
    Eigen::Matrix4d iterative_state_covariance;
    predicted_covariance_.setZero();

    // Add in matched staircase
    int aux_offset = 6 * preceding_measurements_.size(), offset = 4 * preceding_measurements_.size();
    for(int i = 0; i < stair_count_; i++){
        int stateIdx = i;
        Eigen::MatrixXd state_jac(4, 12), param_jacobian(4, 12);
        state_jac.setZero(); param_jacobian.setZero();
        if(matches_from_current_state_.count(stateIdx) != 0){

            int measuremementIdx = matches_from_current_state_[stateIdx];

            std::vector<Eigen::Vector3d> pred_starts;
            std::vector<Eigen::Vector3d> pred_ends;
            Eigen::Vector3d s_start = stair_aux_state_.segment(6*stateIdx, 3), predicted_start; 
            Eigen::Vector3d s_end = stair_aux_state_.segment((6*stateIdx) + 3, 3), predicted_end; 

            Eigen::Vector3d m_start = transform * measurement_steps[measuremementIdx].start_p;
            Eigen::Vector3d m_end = transform * measurement_steps[measuremementIdx].end_p;
            
             // Step 4: Update the corresponding measurements (local measurement vector, registered measurement vector, registered measurement aux state)
            // Also check for inversion in measurement (make sure endpoints are rightly matched) before updating measurment vector;
            float state_line_dir = atan2(s_end(1) - s_start(1), s_end(0) - s_start(0));
            float measure_line_dir = atan2(m_end(1) - m_start(1), m_end(0) - m_start(0));

            if (fabs(stair_utility::wrap2PI(state_line_dir - measure_line_dir)) < M_PI_2) {
                // Average Starts and Ends Together
                registered_input_aux_measurement_.segment(aux_offset, 3) = m_start;
                registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_end;

                input_measurement_vector_.segment(offset, 2) = measurement_steps[measuremementIdx].line_polar_form;
                input_measurement_vector_(offset + 2) = measurement_steps[measuremementIdx].start_p.z();
                input_measurement_vector_(offset + 3) = measurement_steps[measuremementIdx].end_p.z();

                registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[measuremementIdx].line_polar_form;
                registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[measuremementIdx].start_p.z();
                registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[measuremementIdx].end_p.z();
            } 
            else {
                // Average Starts with Ends (reverse the measurement points)
                registered_input_aux_measurement_.segment(aux_offset, 3) = m_end;
                registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_start;

                input_measurement_vector_.segment(offset, 2) = measurement_steps[measuremementIdx].line_polar_form;
                input_measurement_vector_(offset + 2) = measurement_steps[measuremementIdx].end_p.z();
                input_measurement_vector_(offset + 3) = measurement_steps[measuremementIdx].start_p.z();

                registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[measuremementIdx].line_polar_form;
                registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[measuremementIdx].end_p.z();
                registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[measuremementIdx].start_p.z();
            }

            // Step 2: Predict new endpoints of the current stair AND Predict stair state for existing staircase - Use preceeding and succeeding stairs to update prediction
            // Predict state from prev-state and constant-width asSumption
            double line_dir = atan2(s_end(1) - s_start(1), s_end(0) - s_start(0));
            double w_2_cos_theta = 0.5 * staircase_params_(2) * cos(line_dir);
            double w_2_sin_theta = 0.5 * staircase_params_(2) * sin(line_dir);
           
            std::vector<double> radii, angles;
            radii.clear(); angles.clear();

            // Update current step as is
            radii.push_back(stair_state_(4 * stateIdx));
            angles.push_back(stair_state_((4 * stateIdx) + 1));
            
            Eigen::Vector3d temp_s, temp_e;
            temp_s(0) = 0.5 * (s_start(0) + s_end(0)) - (w_2_cos_theta);
            temp_s(1) = 0.5 * (s_start(1) + s_end(1)) - (w_2_sin_theta);
            temp_s(2) = 0.5 * (s_start(2) + s_end(2));
            pred_starts.push_back(temp_s);

            temp_e(0) = 0.5 * (s_start(0) + s_end(0)) + (w_2_cos_theta);
            temp_e(1) = 0.5 * (s_start(1) + s_end(1)) + (w_2_sin_theta);
            temp_e(2) = 0.5 * (s_start(2) + s_end(2));
            pred_ends.push_back(temp_e);

            std::array<double, 3> prev_stair_jacob, next_stair_jacob;
            std::pair<double, double> pred_r_prev, pred_r_next;

            if(i >= 1){
                // Get endpoints and center of nrev_stair
                s_start = stair_aux_state_.segment(6 * (stateIdx - 1), 3);
                s_end = stair_aux_state_.segment(6 * (stateIdx - 1) + 3, 3);
                Eigen::Vector3d center = (s_start + s_end) / 2;

                // Use previous step to add to the current step prediction;
                getSucceedingRadiusAngle(stair_state_(4 * (stateIdx - 1)), stair_state_(4 * (stateIdx - 1) + 1), center.segment(0, 2), i - 1, pred_r_prev, prev_stair_jacob);
                
                float effective_depth_s = (staircase_params_(1) + (0.5 * staircase_params_(2) * sin(staircase_params_(4))));
                float effective_depth_e = (staircase_params_(1) - (0.5 * staircase_params_(2) * sin(staircase_params_(4))));

                int s_multiplier = 1, e_multiplier = 1; // If staircase is curving, endpoints need to be adjusted based on curvature, outer endpoints move more than inner ones.
                if(effective_depth_s/effective_depth_e >= 1.1){
                    // Positive staircase curvature, end moves at slower rotation
                    e_multiplier = 0.5;
                }
                else if (effective_depth_e/effective_depth_s >= 1.1){
                    // Negative staircase curvature, start moves at slower rotation
                    s_multiplier = 0.5;
                }
                else{
                    // Straight staircase, don't apply any rotation
                    s_multiplier = 0.0;
                    e_multiplier = 0.0;
                }

                // Get endpoints of the previous step;
                temp_s(0) = s_start(0) + (effective_depth_s * cos(step_directions_[i - 1] + (s_multiplier * staircase_params_(4))));
                temp_s(1) = s_start(1) + (effective_depth_s * sin(step_directions_[i - 1] + (s_multiplier * staircase_params_(4))));
                temp_s(2) = s_start(2) + staircase_params_(0);

                // temp_s(0) = center(0) + (staircase_params_(1) * cos(step_directions_[i - 1] + staircase_params_(4))) - (0.5 * staircase_params_(2) * cos(step_directions_[i - 1] + staircase_params_(4) + M_PI_2));
                // temp_s(1) = center(1) + (staircase_params_(1) * sin(step_directions_[i - 1] + staircase_params_(4))) - (0.5 * staircase_params_(2) * sin(step_directions_[i - 1] + staircase_params_(4) + M_PI_2));
                // temp_s(2) = center(2) + staircase_params_(0);
                
                temp_e(0) = s_end(0) + (effective_depth_e * cos(step_directions_[i - 1] + (e_multiplier * staircase_params_(4))));
                temp_e(1) = s_end(1) + (effective_depth_e * sin(step_directions_[i - 1] + (e_multiplier * staircase_params_(4))));
                temp_e(2) = s_end(2) + staircase_params_(0);

                // temp_e(0) = center(0) + (staircase_params_(1) * cos(step_directions_[i - 1] + staircase_params_(4))) + (0.5 * staircase_params_(2) * cos(step_directions_[i - 1] + staircase_params_(4) + M_PI_2));
                // temp_e(1) = center(1) + (staircase_params_(1) * sin(step_directions_[i - 1] + staircase_params_(4))) + (0.5 * staircase_params_(2) * sin(step_directions_[i - 1] + staircase_params_(4) + M_PI_2));
                // temp_e(2) = center(2) + staircase_params_(0);

                double new_line_dir = atan2(temp_e(1) - temp_s(1), temp_e(0) - temp_s(0)); 

                if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
                    pred_starts.push_back(temp_s);
                    pred_ends.push_back(temp_e);
                }
                else{
                    pred_starts.push_back(temp_e);
                    pred_ends.push_back(temp_s);
                }   

                // std::cout << "plot3([" << temp_s(0) << "," << temp_e(0) << "], [" << temp_s(1) << "," << temp_e(1) << "], [" << temp_s(2) << "," << temp_e(2) << "], 'LineWidth', 3, 'Color', 'r')" << std::endl;
                // auto new_r_a_endpoints = angleAndRadiusFromEndpoints(pred_starts.back(), pred_ends.back());
                radii.push_back(pred_r_prev.first);
                angles.push_back(pred_r_prev.second);
            }
            if(i < stair_count_ - 1){
                // Get endpoints of curr_stair
                s_start = stair_aux_state_.segment(6 * (stateIdx + 1), 3);
                s_end = stair_aux_state_.segment(6 * (stateIdx + 1) + 3, 3);
                Eigen::Vector3d center = (s_start + s_end) / 2;

                // Use next step to add to the current step prediction;
                getPreceedingRadiusAngle(stair_state_(4 * (stateIdx + 1)), stair_state_(4 * (stateIdx + 1) + 1), center.segment(0, 2), i + 1, pred_r_next, next_stair_jacob);
            
                float effective_depth_s = (staircase_params_(1) + (0.5 * staircase_params_(2) * sin(staircase_params_(4))));
                float effective_depth_e = (staircase_params_(1) - (0.5 * staircase_params_(2) * sin(staircase_params_(4))));
                
                int s_multiplier = 1, e_multiplier = 1; // If staircase is curving, endpoints need to be adjusted based on curvature, outer endpoints move more than inner ones.
                if(effective_depth_s/effective_depth_e >= 1.1){
                    // Positive staircase curvature, end moves at slower rotation
                    e_multiplier = 0.5;
                }
                else if (effective_depth_e/effective_depth_s >= 1.1){
                    // Negative staircase curvature, start moves at slower rotation
                    s_multiplier = 0.5;
                }
                else{
                    // Straight staircase, don't apply any rotation
                    s_multiplier = 0.0;
                    e_multiplier = 0.0;
                }

                // Get endpoints of the next step;
                temp_s(0) = s_start(0) - (effective_depth_s * cos(step_directions_[i + 1] - (s_multiplier * staircase_params_(4))));
                temp_s(1) = s_start(1) - (effective_depth_s * sin(step_directions_[i + 1] - (s_multiplier * staircase_params_(4))));
                temp_s(2) = s_start(2) - staircase_params_(0);

                // temp_s(0) = center(0) - (staircase_params_(1) * cos(step_directions_[i + 1] - staircase_params_(4))) - (0.5 * staircase_params_(2) * cos(step_directions_[i + 1] - staircase_params_(4) + M_PI_2));
                // temp_s(1) = center(1) - (staircase_params_(1) * sin(step_directions_[i + 1] - staircase_params_(4))) - (0.5 * staircase_params_(2) * sin(step_directions_[i + 1] - staircase_params_(4) + M_PI_2));
                // temp_s(2) = center(2) - staircase_params_(0);

                temp_e(0) = s_end(0) - (effective_depth_e * cos(step_directions_[i + 1] - (e_multiplier * staircase_params_(4))));
                temp_e(1) = s_end(1) - (effective_depth_e * sin(step_directions_[i + 1] - (e_multiplier * staircase_params_(4))));
                temp_e(2) = s_end(2) - staircase_params_(0);
                
                // temp_e(0) = center(0) - (staircase_params_(1) * cos(step_directions_[i + 1] - staircase_params_(4))) + (0.5 * staircase_params_(2) * cos(step_directions_[i + 1] - staircase_params_(4) + M_PI_2));
                // temp_e(1) = center(1) - (staircase_params_(1) * sin(step_directions_[i + 1] - staircase_params_(4))) + (0.5 * staircase_params_(2) * sin(step_directions_[i + 1] - staircase_params_(4) + M_PI_2));
                // temp_e(2) = center(2) - staircase_params_(0);
                
                double new_line_dir = atan2(temp_e(1) - temp_s(1), temp_e(0) - temp_s(0)); 

                if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
                    pred_starts.push_back(temp_s);
                    pred_ends.push_back(temp_e);
                }
                else{
                    pred_starts.push_back(temp_e);
                    pred_ends.push_back(temp_s);
                }    
                
                // std::cout << "plot3([" << temp_s(0) << "," << temp_e(0) << "], [" << temp_s(1) << "," << temp_e(1) << "], [" << temp_s(2) << "," << temp_e(2) << "], 'LineWidth', 3, 'Color', 'g')" << std::endl;
                // auto new_r_a_endpoints = angleAndRadiusFromEndpoints(pred_starts.back(), pred_ends.back());

                radii.push_back(pred_r_next.first);
                angles.push_back(pred_r_next.second);

            }
            
            double new_rcs = 0, new_rss = 0, new_r, new_theta;
            predicted_start = Eigen::Vector3d::Zero();
            predicted_end = Eigen::Vector3d::Zero();

            for(int s = 0; s < radii.size(); s++){
                new_rss += (radii[s] * sin(angles[s])/radii.size());
                new_rcs += (radii[s] * cos(angles[s])/radii.size());

                predicted_start += pred_starts[s];
                predicted_end += pred_ends[s];
            }

            predicted_start *= 1.0/radii.size();
            predicted_end  *= 1.0/radii.size();

            new_r = sqrt(new_rcs * new_rcs + new_rss * new_rss);
            new_theta = atan2(new_rss, new_rcs);

            auto new_r_a_endpoints = angleAndRadiusFromEndpoints(predicted_start, predicted_end);

            // std::cout << "plot3([" << predicted_start(0) << "," << predicted_end(0) << "], [" << predicted_start(1) << "," << predicted_end(1) << "], [" << predicted_start(2) << "," << predicted_end(2) << "], 'LineWidth', 3, 'Color', 'w')" << std::endl;
            // std::cout << "R: " << new_r << " T: " << new_theta << " PR: " << new_r_a_endpoints.first << " RT: " << new_r_a_endpoints.second << std::endl;

            predicted_state_(offset) = new_r;
            predicted_state_(offset + 1) = new_theta;
            predicted_state_(offset + 2) = predicted_start(2);
            predicted_state_(offset + 3) = predicted_end(2);


            // Step 4: Compute Jacobians for predicted state
            // Compute Jacobian wrt State
            state_jac(0, 0) = 1.0 / (cos(-staircase_params_(4)) * radii.size());
            state_jac(1, 1) = 1.0 / (cos(-staircase_params_(4)) * radii.size());
            state_jac(2, 2) = 1.0 / (2.0 * radii.size());
            state_jac(3, 3) = 1.0 / (2.0 * radii.size());

            state_jac(0, 4) = 1.0 / radii.size();
            state_jac(1, 5) = 1.0 / radii.size();
            state_jac(2, 6) = 1.0 / (2.0 * radii.size());
            state_jac(3, 7) = 1.0 / (2.0 * radii.size());
            
            state_jac(0, 8) = 1.0 / (cos(staircase_params_(4)) * radii.size());
            state_jac(1, 9) = 1.0 / (cos(staircase_params_(4)) * radii.size());
            state_jac(2, 10) = 1.0 / (2.0 * radii.size());
            state_jac(3, 11) = 1.0 / (2.0 * radii.size());

            param_jacobian(0, 1) = prev_stair_jacob[0];
            param_jacobian(0, 4) = prev_stair_jacob[1];
            param_jacobian(1, 4) = prev_stair_jacob[2];
            param_jacobian(2, 0) = -1.0;
            param_jacobian(3, 0) = -1.0;

            param_jacobian(0, 7) = next_stair_jacob[0];
            param_jacobian(0, 10) = next_stair_jacob[1];
            param_jacobian(1, 10) = next_stair_jacob[2];
            param_jacobian(2, 6) = 1.0;
            param_jacobian(3, 6) = 1.0;
            // Update predicted_covariance for this stateIdx

            Eigen::MatrixXd param_sigma_block = 1.0 * staircase_model_sigmas_.asDiagonal();

            predicted_covariance_.block(offset, offset, 4, 4) = (state_jac.block(0, 4, 4, 4) * stair_covariance_.block(4 * stateIdx, 4 * stateIdx, 4, 4) * state_jac.block(0, 4, 4, 4).transpose());
            
            if(i >= 1){
                predicted_covariance_.block(offset, offset, 4, 4) += (state_jac.block(0, 0, 4, 4) * stair_covariance_.block(4 * (stateIdx - 1), 4 * (stateIdx - 1), 4, 4) * state_jac.block(0, 0, 4, 4).transpose()) + (param_jacobian.block(0, 0, 4, 6) * param_sigma_block * param_jacobian.block(0, 0, 4, 6).transpose());
            }
            if(i < stair_count_ - 1){
                predicted_covariance_.block(offset, offset, 4, 4) += (state_jac.block(0, 8, 4, 4) * stair_covariance_.block(4 * (stateIdx + 1), 4 * (stateIdx + 1), 4, 4) * state_jac.block(0, 8, 4, 4).transpose()) + (param_jacobian.block(0, 6, 4, 6) * param_sigma_block * param_jacobian.block(0, 6, 4, 6).transpose());
            }

            // Step 5: Compute the endpoints that maximizes the width as both the measurement or the prediction can be partial
            std::pair<Eigen::Vector3d, Eigen::Vector3d> longestSegment;
            std::vector<Eigen::Vector3d> points = {m_start, m_end, predicted_start, predicted_end}; 
            double maxDistanceSquared = 0.0;
            for (size_t loop1 = 0; loop1 < points.size(); ++loop1) {
                for (size_t loop2 = loop1 + 1; loop2 < points.size(); ++loop2) {
                    double distSq = (points[loop1].segment(0, 2) - points[loop2].segment(0, 2)).squaredNorm();

                    if (distSq > maxDistanceSquared) {
                        maxDistanceSquared = distSq;
                        longestSegment = std::make_pair(points[loop1], points[loop2]);
                    }
                }
            }

            double new_line_dir = atan2(longestSegment.second.y() - longestSegment.first.y(), longestSegment.second.x() - longestSegment.first.x()); 
            if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
                predicted_aux_state_.segment(aux_offset, 3) = longestSegment.first;
                predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.second;
            }
            else{
                predicted_aux_state_.segment(aux_offset, 3) = longestSegment.second;
                predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.first;
            }

        }
        else{

            // When No measurement is received for a state, we just mark the active_measurements_ block for that stair as zero which disables it in the ekf update process.
            predicted_state_.segment(offset, 4) = stair_state_.segment(4 * stateIdx, 4);
            predicted_aux_state_.segment(aux_offset, 6) = stair_aux_state_.segment(6 * stateIdx, 6);
            
            // Set Input measurements to zero
            input_measurement_vector_.segment(offset, 4) = Eigen::VectorXd::Zero(4);
            registered_input_measurement_.segment(offset, 4) = Eigen::VectorXd::Zero(4);

            // Update active_measurements_ matrix to disable the part of the state in EKF update
            active_measurements_.block(offset, offset, 4, 4) = Eigen::MatrixXd::Zero(4, 4);

            predicted_covariance_.block(offset, offset, 4, 4) = stair_covariance_.block(4 * stateIdx, 4 * stateIdx, 4, 4); //Covariance stays the same as no change in state
        }

        aux_offset += 6;
        offset += 4;
    }

    // Add in preceeding measurements
    aux_offset = 6 * (preceding_measurements_.size() - 1), offset = 4 * (preceding_measurements_.size() - 1);
    int j = 1;
    for(auto it = preceding_measurements_.rbegin(); it != preceding_measurements_.rend(); ++it){
        int idx = *it;

        // Step 1: Need to predict preceeding staircase locations(x, y, z) from the bottom-most staircase 
        Eigen::Vector3d center = (stair_aux_state_.segment(0, 3) + stair_aux_state_.segment(3, 3)) / 2.0, predicted_start, predicted_end;

        double cos_theta, sin_theta, cos_theta_pi_2, sin_theta_pi_2;
        cos_theta = cos(staircase_params_(3) - j*staircase_params_(4));
        sin_theta = sin(staircase_params_(3) - j*staircase_params_(4));
        cos_theta_pi_2 = cos(staircase_params_(3) - j*staircase_params_(4) + M_PI_2);
        sin_theta_pi_2 = sin(staircase_params_(3) - j*staircase_params_(4) + M_PI_2);
    
        predicted_start(0) = center(0) - (j*staircase_params_(1)*cos_theta) - (0.5 * staircase_params_(2) * cos_theta_pi_2);
        predicted_start(1) = center(1) - (j*staircase_params_(1)*sin_theta) - (0.5 * staircase_params_(2) * sin_theta_pi_2);
        predicted_start(2) = center(2) - (j*staircase_params_(0));

        predicted_end(0) = center(0) - (j*staircase_params_(1)*cos_theta) + (0.5 * staircase_params_(2) * cos_theta_pi_2);
        predicted_end(1) = center(1) - (j*staircase_params_(1)*sin_theta) + (0.5 * staircase_params_(2) * sin_theta_pi_2);
        predicted_end(2) = center(2) - (j*staircase_params_(0));

        // Step 2: Predict radius and angle state from the current state - and the locations
        int sign1 = 1, sign2 = 1, sign3 = 1;
        if(fabs(stair_utility::wrap2PI(staircase_params_(3) - stair_state_(1))) > M_PI_2){ // Add or remove from Radius State based on staircase direction
            sign1 = -1;
        }

        double curr_r, curr_theta;
        if(j == 1){
            curr_r = stair_state_(0);
            curr_theta = stair_state_(1);
        }
        else{
            curr_r = predicted_state_(offset + 4);
            curr_theta = predicted_state_(offset + 5);
        }

        double new_theta = stair_utility::wrap2PI(curr_theta - staircase_params_(4)); // Compute new angle for the stair

        double cos_delta = cos(-staircase_params_(4));
        double r_pr =  curr_r / cos_delta;
        Eigen::Vector2d r_point = {r_pr * cos(new_theta), r_pr * sin(new_theta)};  // Point on the line that is given by the (r, theta) of the state
        Eigen::Vector2d r_0_point = {curr_r*cos(curr_theta), curr_r*sin(curr_theta)};

        double len = (r_point - center.segment(0, 2)).norm(); // Distance from the center of the lowermost stair to the r_point
        double len2 = (r_0_point - center.segment(0, 2)).norm();
        
        if((len > len2 && staircase_params_(4) < 0) || (len < len2 && staircase_params_(4) > 0)){
            // std::cout << "len1 :" << len << " len2: " << len2 << " delt:" << staircase_params_(4) <<  std::endl;
            // std::cout << "Sign3 switched directions" << std::endl;
            sign3 = -1;
        }

        double new_d = r_pr - (sign1 * (staircase_params_(1) + (len * sin(sign1 * sign3 * staircase_params_(4))))); // Compute new radius for the stair
        
        if(new_d < 0){
            sign2 = -1;
            new_d = -new_d;
            new_theta = stair_utility::wrap2PI(new_theta + M_PI);
        }

        auto new_r_a_endpoints = angleAndRadiusFromEndpoints(predicted_start, predicted_end); // Predicted radius and angle from the locations (Close to the original, but harder to compute jacobian for)
        if(fabs(new_r_a_endpoints.first - new_d) > 0.05 || fabs(new_r_a_endpoints.second - new_theta) > 0.05){
            std::cout << "Mismatch in r-theta prediction for preceeding" << std::endl; // Mismatch or divergence can occur when j*del-theta goes above 90 degrees (Likely shouldn't happen in theory)
            // std::cout <<"j: " << j << " Point Pred, R :" << new_r_a_endpoints.first << " A: " << new_r_a_endpoints.second << " Rad pred, R: " << new_d << " A: " << new_theta << std::endl;
        }

        predicted_state_(offset) = new_d;
        predicted_state_(offset + 1) = new_theta;
        predicted_state_(offset + 2) = predicted_start(2);
        predicted_state_(offset + 3) = predicted_end(2);

        
        // Step 3: Compute Jacobians and update covariances
        // Compute Jacobian wrt to Current States
        Eigen::Matrix4d state_jac;
        Eigen::Matrix<double, 4, 6> param_jac;
        
        state_jac.setIdentity();
        param_jac.setZero();

        state_jac(0, 0) = 1/cos_delta;
        state_jac.block(2, 2, 2, 2) = 0.5 * Eigen::Matrix2d::Identity();

        // Compute Jacobian wrt Model Parameter
        param_jac(0, 1) = -sign1 * sign2;
        param_jac(0, 4) = -sign3 * sign1 * sign2 * len * cos_delta + r_pr * tan(staircase_params_(4));
        param_jac(1, 4) = -1.0 * sign1;
        param_jac(2, 3) =  1.0; 
        param_jac(2, 0) = -1.0;
        param_jac(3, 0) = -1.0;

        // Step 3.5: Update covariances
        if(j == 1){
            iterative_state_covariance = (state_jac * stair_covariance_.block(0, 0, 4, 4) * state_jac.transpose());
            predicted_covariance_.block(offset, offset, 4, 4) =  iterative_state_covariance + (param_jac * staircase_model_sigmas_.asDiagonal() * param_jac.transpose());
        }
        else{

            iterative_state_covariance = (state_jac * iterative_state_covariance * state_jac.transpose()).eval();
            predicted_covariance_.block(offset, offset, 4, 4) =  iterative_state_covariance + (param_jac * staircase_model_sigmas_.asDiagonal() * param_jac.transpose());
        }
        
        // Step 4: Update the corresponding measurements (local measurement vector, registered measurement vector, registered measurement aux state)
        // Also check for inversion in measurement (make sure endpoints are rightly matched) before updating measurment vector;
        Eigen::Vector3d m_start = transform * measurement_steps[idx].start_p;
        Eigen::Vector3d m_end = transform * measurement_steps[idx].end_p;
        float state_line_dir = atan2(predicted_end(1) - predicted_start(1), predicted_end(0) - predicted_start(0));
        float measure_line_dir = atan2(m_end(1) - m_start(1), m_end(0) - m_start(0));

        if (fabs(stair_utility::wrap2PI(state_line_dir - measure_line_dir)) < M_PI_2) {
            // Average Starts and Ends Together
            registered_input_aux_measurement_.segment(aux_offset, 3) = m_start;
            registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_end;

            input_measurement_vector_.segment(offset, 2) = measurement_steps[idx].line_polar_form;
            input_measurement_vector_(offset + 2) = measurement_steps[idx].start_p.z();
            input_measurement_vector_(offset + 3) = measurement_steps[idx].end_p.z();

            registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[idx].line_polar_form;
            registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[idx].start_p.z();
            registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[idx].end_p.z();
        } 
        else {
            // Average Starts with Ends (reverse the measurement points)
            registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_start;
            registered_input_aux_measurement_.segment(aux_offset, 3) = m_end;

            input_measurement_vector_.segment(offset, 2) = measurement_steps[idx].line_polar_form;
            input_measurement_vector_(offset + 2) = measurement_steps[idx].end_p.z();
            input_measurement_vector_(offset + 3) = measurement_steps[idx].start_p.z();

            registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[idx].line_polar_form;
            registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[idx].end_p.z();
            registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[idx].start_p.z();
        }
    
        // Step 5: Compute the endpoints that maximizes the width as both the measurement or the prediction can be partial
        std::pair<Eigen::Vector3d, Eigen::Vector3d> longestSegment;
        std::vector<Eigen::Vector3d> points = {m_start, m_end, predicted_start, predicted_end}; 
        double maxDistanceSquared = 0.0;
        for (size_t loop1 = 0; loop1 < points.size(); ++loop1) {
            for (size_t loop2 = loop1 + 1; loop2 < points.size(); ++loop2) {
                double distSq = (points[loop1].segment(0, 2) - points[loop2].segment(0, 2)).squaredNorm();

                if (distSq > maxDistanceSquared) {
                    maxDistanceSquared = distSq;
                    longestSegment = std::make_pair(points[loop1], points[loop2]);
                }
            }
        }

        double new_line_dir = atan2(longestSegment.second.y() - longestSegment.first.y(), longestSegment.second.x() - longestSegment.first.x()); 
        if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
            predicted_aux_state_.segment(aux_offset, 3) = longestSegment.first;
            predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.second;
        }
        else{
            predicted_aux_state_.segment(aux_offset, 3) = longestSegment.second;
            predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.first;
        }

        j += 1;
        aux_offset -= 6;
        offset -= 4;
    }


    // Add in succeeding measurements;
    aux_offset = 6 * (stair_count_ + preceding_measurements_.size()), offset = 4 * (stair_count_ + preceding_measurements_.size());
    j = 1;
    for(auto it = succeeding_measurements_.begin(); it != succeeding_measurements_.end(); ++it){
        int idx = *it;

        // Step 1: Need to predict succeeding staircase locations(x, y, z) from the top-most staircase 
        Eigen::Vector3d center = (stair_aux_state_.segment(6*(stair_count_ - 1), 3) + stair_aux_state_.segment(6*(stair_count_ - 1) + 3, 3)) / 2.0, predicted_start, predicted_end;
        
        double cos_theta, sin_theta, cos_theta_pi_2, sin_theta_pi_2;
        cos_theta = cos(staircase_params_(5) + j*staircase_params_(4));
        sin_theta = sin(staircase_params_(5) + j*staircase_params_(4));
        cos_theta_pi_2 = cos(staircase_params_(5) + j*staircase_params_(4) + M_PI_2);
        sin_theta_pi_2 = sin(staircase_params_(5) + j*staircase_params_(4) + M_PI_2);

        predicted_start(0) = center(0) + (j*staircase_params_(1)*cos_theta) - (0.5 * staircase_params_(2) * cos_theta_pi_2);
        predicted_start(1) = center(1) + (j*staircase_params_(1)*sin_theta) - (0.5 * staircase_params_(2) * sin_theta_pi_2);
        predicted_start(2) = center(2) + (j*staircase_params_(0));

        predicted_end(0) = center(0) + (j*staircase_params_(1)*cos_theta) + (0.5 * staircase_params_(2) * cos_theta_pi_2);
        predicted_end(1) = center(1) + (j*staircase_params_(1)*sin_theta) + (0.5 * staircase_params_(2) * sin_theta_pi_2);
        predicted_end(2) = center(2) + (j*staircase_params_(0));

        // Step 2: Predict radius and angle state from the current state - and the locations
        int sign1 = 1, sign2 = 1, sign3 = 1;
        if(fabs(stair_utility::wrap2PI(staircase_params_(5) - stair_state_(4 * (stair_count_ - 1) + 1))) > M_PI_2){ // Add or remove from Radius State based on staircase direction
            sign1 = -1;
        }

        double curr_r, curr_theta;
        if(j == 1){
            curr_r = stair_state_(4 * (stair_count_ - 1));
            curr_theta = stair_state_(4 * (stair_count_ - 1) + 1);
        }
        else{
            curr_r = predicted_state_(offset - 4);
            curr_theta = predicted_state_(offset - 3);
        }

        double new_theta = stair_utility::wrap2PI(curr_theta + staircase_params_(4)); // Compute new angle for the stair
        
        double cos_delta = cos(staircase_params_(4));
        double r_pr =  curr_r / cos_delta;

        Eigen::Vector2d r_point = {r_pr * cos(new_theta), r_pr * sin(new_theta)};  // Point on the line that is given by the (r, theta) of the state
        Eigen::Vector2d r_0_point = {curr_r * cos(curr_theta), curr_r * sin(curr_theta)};

        double len = (r_point - center.segment(0, 2)).norm(); // Distance from the center of the lowermost stair to the r_point
        double len2 = (r_0_point - center.segment(0, 2)).norm();
        
        if((len > len2 && staircase_params_(4) < 0) || (len < len2 && staircase_params_(4) > 0)){
            // std::cout << "len1 :" << len << " len2: " << len2 << " delt:" << staircase_params_(4) <<  std::endl;
            // std::cout << "Sign3 switched directions" << std::endl;
            sign3 = -1;
        }

        double new_d = r_pr + (sign1 * (staircase_params_(1) - (len * sin(sign3 * sign1 * staircase_params_(4))))); // Compute new radius for the stair

        if(new_d < 0){
            sign2 = -1;
            new_d = -new_d;
            new_theta = stair_utility::wrap2PI(new_theta + M_PI);
        }

        auto new_r_a_endpoints = angleAndRadiusFromEndpoints(predicted_start, predicted_end); // Predicted radius and angle from the locations (Close to the original, but harder to compute jacobian for)
        if(fabs(new_r_a_endpoints.first - new_d) > 0.05 || fabs(new_r_a_endpoints.second - new_theta) > 0.05){
            std::cout << "Mismatch in r-theta prediction for succeeding!" << std::endl; // Mismatch or divergence can occur when j*del-theta goes to 90 degrees (Likely shouldn't happen in theory)
            // std::cout << "j: " << j << " Point Pred, R :" << new_r_a_endpoints.first << " A: " << new_r_a_endpoints.second << " Rad pred, R: " << new_d << " A: " << new_theta << std::endl;
        }

        predicted_state_(offset) = new_d;
        predicted_state_(offset + 1) = new_theta;
        predicted_state_(offset + 2) = predicted_start(2);
        predicted_state_(offset + 3) = predicted_end(2);


        // Step 3: Compute Jacobians and update covariances
        Eigen::Matrix4d state_jac;
        Eigen::Matrix<double, 4, 6> param_jac;
        state_jac.setIdentity();
        param_jac.setZero();

        // Compute Jacobian wrt to Current States
        state_jac(0, 0) = 1/cos_delta;
        state_jac.block(2, 2, 2, 2) = 0.5 * Eigen::Matrix2d::Identity();

        // Compute Jacobian wrt Model parameters
        param_jac(0, 1) =  sign1 * sign2;
        param_jac(0, 4) =  sign3 * sign1 * sign2 * len * cos_delta + r_pr * tan(staircase_params_(4));
        param_jac(1, 4) =  sign1;
        param_jac(2, 5) =  1; 
        param_jac(2, 0) =  1;
        param_jac(3, 0) =  1;

        // Update covariances
        if(j == 1){
            iterative_state_covariance = (state_jac * stair_covariance_.block(4 * (stair_count_ - 1), 4 * (stair_count_ - 1), 4, 4) * state_jac.transpose());
            predicted_covariance_.block(offset, offset, 4, 4) = iterative_state_covariance + (param_jac * staircase_model_sigmas_.asDiagonal() * param_jac.transpose());
        }
        else{
            iterative_state_covariance = (state_jac * iterative_state_covariance * state_jac.transpose()).eval();
            predicted_covariance_.block(offset, offset, 4, 4) =  iterative_state_covariance + (param_jac * staircase_model_sigmas_.asDiagonal() * param_jac.transpose());
        }

        // Step 4: Update the corresponding measurements (local measurement vector, registered measurement vector, registered measurement aux state)
        // Also check for inversion in measurement (make sure endpoints are rightly matched) before updating measurment vector;
        Eigen::Vector3d m_start = transform * measurement_steps[idx].start_p;
        Eigen::Vector3d m_end = transform * measurement_steps[idx].end_p;
        float state_line_dir = atan2(predicted_end(1) - predicted_start(1), predicted_end(0) - predicted_start(0));
        float measure_line_dir = atan2(m_end(1) - m_start(1), m_end(0) - m_start(0));

        if (fabs(stair_utility::wrap2PI(state_line_dir - measure_line_dir)) < M_PI_2) {
            registered_input_aux_measurement_.segment(aux_offset, 3) = m_start;
            registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_end;

            input_measurement_vector_.segment(offset, 2) = measurement_steps[idx].line_polar_form;
            input_measurement_vector_(offset + 2) = measurement_steps[idx].start_p.z();
            input_measurement_vector_(offset + 3) = measurement_steps[idx].end_p.z();

            registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[idx].line_polar_form;
            registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[idx].start_p.z();
            registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[idx].end_p.z();

        } else {
            registered_input_aux_measurement_.segment(aux_offset + 3, 3) = m_start;
            registered_input_aux_measurement_.segment(aux_offset, 3) = m_end;

            input_measurement_vector_.segment(offset, 2) = measurement_steps[idx].line_polar_form;
            input_measurement_vector_(offset + 2) = measurement_steps[idx].end_p.z();
            input_measurement_vector_(offset + 3) = measurement_steps[idx].start_p.z();

            registered_input_measurement_.segment(offset, 2) = new_staircase_registered_.steps[idx].line_polar_form;
            registered_input_measurement_(offset + 2) = new_staircase_registered_.steps[idx].end_p.z();
            registered_input_measurement_(offset + 3) = new_staircase_registered_.steps[idx].start_p.z();
        }

        // Step 5: Compute the endpoints that maximizes the width as both the measurement or the prediction can be partial
        std::pair<Eigen::Vector3d, Eigen::Vector3d> longestSegment;
        std::vector<Eigen::Vector3d> points = {m_start, m_end, predicted_start, predicted_end}; 
        double maxDistanceSquared = 0.0;
        for (size_t loop1 = 0; loop1 < points.size(); ++loop1) {
            for (size_t loop2 = loop1 + 1; loop2 < points.size(); ++loop2) {
                double distSq = (points[loop1].segment(0, 2) - points[loop2].segment(0, 2)).squaredNorm();

                if (distSq > maxDistanceSquared) {
                    maxDistanceSquared = distSq;
                    longestSegment = std::make_pair(points[loop1], points[loop2]);
                }
            }
        }

        double new_line_dir = atan2(longestSegment.second.y() - longestSegment.first.y(), longestSegment.second.x() - longestSegment.first.x()); 
        if (fabs(stair_utility::wrap2PI(state_line_dir - new_line_dir)) < M_PI_2) {
            predicted_aux_state_.segment(aux_offset, 3) = longestSegment.first;
            predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.second;
        }
        else{
            predicted_aux_state_.segment(aux_offset, 3) = longestSegment.second;
            predicted_aux_state_.segment(aux_offset + 3, 3) = longestSegment.first;
        }


        j += 1;
        offset += 4;
        aux_offset += 6;
    }

}

void StaircaseModel::transformPredictedStatetoLocalFrame(const Eigen::Affine3d& transform){
    
    Eigen::MatrixXd measurement_to_state_jac, measurement_to_pose_jac;
    measurement_to_state_jac.setIdentity(predicted_state_.rows(), predicted_state_.rows());
    measurement_to_pose_jac.setZero(predicted_state_.rows(), predicted_state_.rows());
    predicted_measurement_vector_.resizeLike(predicted_state_);

    Eigen::Affine3d inverse_transform = transform.inverse();
    int total_count = predicted_state_.size() / 4;
    
    double tx, ty, tz, r_theta;
    r_theta = inverse_transform.rotation().eulerAngles(0, 1, 2)[2];
    tx = inverse_transform.translation().x();
    ty = inverse_transform.translation().y();
    tz = inverse_transform.translation().z();

    for(int i = 0; i < total_count; i++){
        
        double d = predicted_state_(4 * i);
        double phi = predicted_state_(4 * i + 1);

        // Compute Expected Measurement
        double new_d = d + tx * cos(phi + r_theta) + ty * sin(phi + r_theta);
        if(new_d < 0){
            predicted_measurement_vector_(4 * i) = -new_d;
            predicted_measurement_vector_(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta + M_PI);
            // std::cout << "HERE IN MEasurement R NEGATIVE" << std::endl;

            // Compute Jacobian wrt to state terms
            measurement_to_state_jac(4 * i, 4 * i + 1) = -(ty * cos(phi + r_theta) - tx * sin(phi + r_theta)); 
            
            // Compute Jacobian wrt to pose terms
            measurement_to_pose_jac(4 * i, 4 * i) = cos(stair_utility::wrap2PI(phi + r_theta + M_PI));
            measurement_to_pose_jac(4 * i, 4 * i + 1) = sin(stair_utility::wrap2PI(phi + r_theta + M_PI));
            measurement_to_pose_jac(4 * i + 1, 4 * i + 3) = 1;
            measurement_to_pose_jac(4 * i + 2, 4 * i + 2) = 1;
            measurement_to_pose_jac(4 * i + 3, 4 * i + 2) = 1; 
        }
        else{
            predicted_measurement_vector_(4 * i) = new_d;
            predicted_measurement_vector_(4 * i + 1) = stair_utility::wrap2PI(phi + r_theta);

             // Compute Jacobian wrt to state terms
            measurement_to_state_jac(4 * i, 4 * i + 1) = ty * cos(phi + r_theta) - tx * sin(phi + r_theta); 
            
            // Compute Jacobian wrt to pose terms
            measurement_to_pose_jac(4 * i, 4 * i) = cos(phi + r_theta);
            measurement_to_pose_jac(4 * i, 4 * i + 1) = sin(phi + r_theta);
            measurement_to_pose_jac(4 * i + 1, 4 * i + 3) = 1;
            measurement_to_pose_jac(4 * i + 2, 4 * i + 2) = 1;
            measurement_to_pose_jac(4 * i + 3, 4 * i + 2) = 1; 
        }
       
       
        predicted_measurement_vector_(4 * i + 2) = predicted_state_(4 * i + 2) + tz;
        predicted_measurement_vector_(4 * i + 3) = predicted_state_(4 * i + 3) + tz;    

    }

    Eigen::MatrixXd pose_sigma_block = pose_sigmas_.replicate(total_count, 1).asDiagonal();
    predicted_local_covariance_ = (measurement_to_state_jac * predicted_covariance_ * measurement_to_state_jac.transpose()).eval() + (measurement_to_pose_jac * pose_sigma_block * measurement_to_pose_jac.transpose()).eval();

}

stair_utility::StaircaseProcessingResult StaircaseModel::updateStaircase(const stair_utility::StaircaseMeasurement& new_staircase){

    stair_utility::StaircaseProcessingResult process_result;
    process_result.success = true;

    // Count this as another world-frame observation of the same staircase.
    times_observed_++;

    std::chrono::system_clock::time_point t1, t2, t3, t4, t5, t6, t7, t8;
    t1 = std::chrono::high_resolution_clock::now();
    
    // Step 0: Convert Measurement to Global Frame

    new_staircase_registered_.stair_count = new_staircase.stair_count;
   
    Eigen::Affine3d transform = new_staircase.robot_pose.vehicle_pos * new_staircase.robot_pose.vehicle_quat;
    double tx, ty, tz, r_theta;
    tx = new_staircase.robot_pose.vehicle_pos.x();
    ty = new_staircase.robot_pose.vehicle_pos.y();
    tz = new_staircase.robot_pose.vehicle_pos.z();
    r_theta = new_staircase.robot_pose.vehicle_quat.toRotationMatrix().eulerAngles(0, 1, 2)[2];

    new_staircase_registered_.steps.clear();
    for(int i = 0; i < new_staircase.stair_count; i++){
        stair_utility::StairStep step;
        step.start_p = transform * new_staircase.steps[i].start_p;
        step.end_p = transform * new_staircase.steps[i].end_p;
        step.step_width = new_staircase.steps[i].step_width;
        
        double new_d = new_staircase.steps[i].line_polar_form(0) + tx * cos(new_staircase.steps[i].line_polar_form(1) + r_theta) + ty * sin(new_staircase.steps[i].line_polar_form(1) + r_theta);
        double new_phi = stair_utility::wrap2PI(new_staircase.steps[i].line_polar_form(1) + r_theta);

        Eigen::Matrix4d measurement_jacobian, pose_jacobian;
        measurement_jacobian.setIdentity();
        pose_jacobian.setZero();
            
        if(new_d > 0){
            step.line_polar_form(0) = new_d;
            step.line_polar_form(1) = new_phi;

            measurement_jacobian(0, 1) = ty * cos(new_phi) - tx * sin(new_phi); 

            pose_jacobian.row(0) << cos(new_phi), sin(new_phi), 0, 0;
            pose_jacobian(1, 3) = 1;
            pose_jacobian(2, 2) = 1;
            pose_jacobian(3, 2) = 1; 

        }
        else{
            step.line_polar_form(0) = -new_d;
            step.line_polar_form(1) = stair_utility::wrap2PI(new_phi + M_PI);
            measurement_jacobian(0, 1) = - (ty * cos(new_phi) - tx * sin(new_phi)); 

            pose_jacobian.row(0) << cos(step.line_polar_form(1)), sin(step.line_polar_form(1)), 0, 0;
            pose_jacobian(1, 3) = 1;
            pose_jacobian(2, 2) = 1;
            pose_jacobian(3, 2) = 1; 

        }
        step.step_covariance = pose_jacobian * init_pose_sigmas_.asDiagonal() * pose_jacobian.transpose() + measurement_jacobian * new_staircase.steps[i].step_covariance * measurement_jacobian.transpose();

        new_staircase_registered_.steps.push_back(step);
    }
        
    if(filter_type_ == stair_utility::StaircaseFilterType::SimpleAveraging || filter_type_ == stair_utility::StaircaseFilterType::SimpleMaximum){
        // Step 1: Find matches between new_staircase and current state
        t2 = std::chrono::high_resolution_clock::now();

        bool result = computeMeasurementMatches(new_staircase_registered_.stair_count, new_staircase_registered_.steps);
        if(!result){
            process_result.success = false;
            return process_result;
        }
           
        t3 = std::chrono::high_resolution_clock::now();
        t4 = std::chrono::high_resolution_clock::now();
        t5 = std::chrono::high_resolution_clock::now();
        // Step 2: Perform averaging or maximize based on filter type
        if(filter_type_ == stair_utility::StaircaseFilterType::SimpleAveraging){
            applyAveragingFilter(new_staircase_registered_.stair_count, new_staircase_registered_.steps);
        }
        else if (filter_type_ == stair_utility::StaircaseFilterType::SimpleMaximum){
            applyMaximizingFilter(new_staircase_registered_.stair_count, new_staircase_registered_.steps);
        }
    }
    else if(filter_type_ == stair_utility::StaircaseFilterType::LocalFrameEKF){
        
        // Step 1: Find matches between new_staircase and current state
        // bool result = computeMeasurementMatches(new_staircase_registered_.stair_count, new_staircase_registered_.steps);
        t2 = std::chrono::high_resolution_clock::now();

        bool result = computeMeasurementMatchesWithCovarianceInLocalFrame(new_staircase_registered_.stair_count, new_staircase.steps, transform);
        if(!result){
            result = computeMeasurementMatches(new_staircase_registered_.stair_count, new_staircase_registered_.steps);
            if(!result){
                process_result.success = false;
                return process_result;
            }
        }
        
        t3 = std::chrono::high_resolution_clock::now();
        
        std::cout << "\033[0;36m[Stair EKF (Global/Local)] Predicting \033[0m" << std::endl; 

        // Step 2, Use the process function to predict where the expected staircase state would be, and its corresponding covariances
        predictStaircaseStateFromModel(new_staircase.stair_count, new_staircase.steps, transform);

        t4 = std::chrono::high_resolution_clock::now();

        t5 = std::chrono::high_resolution_clock::now();
        // Step 3, get the expected measurement and its covariance from the expected state.

        std::cout << "\033[0;36m[Stair EKF Local] Transforming Prediction to Local Frame \033[0m" << std::endl; 
        transformPredictedStatetoLocalFrame(transform);
        
        // Step 4, apply ekf to get new state
        int num_stairs = predicted_measurement_vector_.rows() / 4;
        std::cout << "\033[0;36m[Stair EKF Local] Applying EKF in Local Frame \033[0m" << std::endl; 

        residual_vector_ = input_measurement_vector_ - predicted_measurement_vector_;
        for(int i = 0; i < num_stairs; i++){
            residual_vector_(4 * i + 1) = stair_utility::wrap2PI(residual_vector_(4 * i + 1));
        }
        
        // Check for outliers in measurements, unless stair is part of preceeding or succeeding measurement
        // Outlier detection is performeed using Mahalanobis distance 
        Eigen::MatrixXd Hk = Eigen::MatrixXd::Identity(num_stairs * 4, num_stairs * 4);
        Hk = (active_measurements_ * Hk).eval();
        
        Eigen::MatrixXd measure_noise = (measurement_sigmas_.replicate(num_stairs, 1).asDiagonal());
        residual_covariance_ = (Hk * predicted_local_covariance_ * Hk.transpose()).eval() + measure_noise;

        for(int i = 0; i < num_stairs; i++){
            if(i >= preceding_measurements_.size() && i < (preceding_measurements_.size() + stair_count_)){
                Eigen::Vector2d curr_res = active_measurements_.block(4 * i, 4 * i, 2, 2) * residual_vector_.segment(4 * i, 2);
                Eigen::Matrix2d curr_innv = residual_covariance_.block(4 * i, 4 * i, 2, 2);
                double d = sqrt(curr_res.transpose() * curr_innv.inverse() * curr_res);
        
                if(d > 4){
                    // std::cout << "Outlier in step " << i + 1 << "/" << num_stairs << "Mahalanobis Distance: " << d << std::endl;
                    active_measurements_.block(4 * i, 4 * i, 4, 4) = Eigen::Matrix4d::Zero();
                }
            }
        }

        Hk = (active_measurements_ * Hk).eval();
        residual_covariance_ = (Hk * predicted_local_covariance_ * Hk.transpose()).eval() + measure_noise;

        kalman_gain_ = predicted_local_covariance_ * Hk.transpose() * residual_covariance_.inverse();
        // std::cout << kalman_gain_ << std::endl;
        
        // Update the covariance
        Eigen::MatrixXd temp = kalman_gain_ * Hk;
        StaircaseStateCovariance new_local_cov = (Eigen::MatrixXd::Identity(temp.rows(), temp.cols()) - temp) * predicted_local_covariance_;

        // Apply EKF in the local frame, and then transform the state back to inertial 
        StaircaseState temp_local_state = predicted_measurement_vector_ + kalman_gain_ * residual_vector_;
        
        stair_count_ = temp_local_state.size() / 4;

        // Transform update back to global frame, update aux state, and reproject on line, as well as ensure r, and theta are bounded
        stair_state_.resize(4 * stair_count_);
        stair_covariance_.resize(4 * stair_count_, 4 * stair_count_);
        stair_aux_state_.resize(6 * stair_count_);
        for(int i = 0; i < stair_count_; i++){
            double d = temp_local_state(4 * i);
            double phi = temp_local_state(4 * i + 1);

            double new_d = d + tx * cos(phi + r_theta) + ty * sin(phi + r_theta);
            double new_phi = stair_utility::wrap2PI(phi + r_theta);

            Eigen::Matrix4d measurement_jacobian, pose_jacobian;
            measurement_jacobian.setIdentity();
            pose_jacobian.setZero();

            if(new_d > 0){
                stair_state_(4 * i) = new_d;
                stair_state_(4 * i + 1) = new_phi;

                measurement_jacobian(0, 1) = ty * cos(new_phi) - tx * sin(new_phi); 

                pose_jacobian.row(0) << cos(new_phi), sin(new_phi), 0, 0;
                pose_jacobian(1, 3) = 1;
                pose_jacobian(2, 2) = 1;
                pose_jacobian(3, 2) = 1; 
            }
            else{
                stair_state_(4 * i) = -new_d;
                stair_state_(4 * i + 1) = stair_utility::wrap2PI(new_phi + M_PI);

                measurement_jacobian(0, 1) = - (ty * cos(new_phi) - tx * sin(new_phi)); 

                pose_jacobian.row(0) << cos(stair_state_(4 * i + 1)), sin(stair_state_(4 * i + 1)), 0, 0;
                pose_jacobian(1, 3) = 1;
                pose_jacobian(2, 2) = 1;
                pose_jacobian(3, 2) = 1; 
            }
            
            stair_state_(4 * i + 2) = temp_local_state(4 * i + 2) + tz;
            stair_state_(4 * i + 3) = temp_local_state(4 * i + 3) + tz;

            stair_covariance_.block(4 * i, 4 * i, 4, 4) = (measurement_jacobian * new_local_cov.block(4 * i, 4 * i, 4, 4) * measurement_jacobian.transpose()) +  (pose_jacobian * pose_sigmas_.asDiagonal() * pose_jacobian.transpose());

            Eigen::Vector2d new_start, new_end;
            projectEndpoints(stair_state_(4 * i), stair_state_(4 * i + 1), predicted_aux_state_.segment(6 * i, 2), predicted_aux_state_.segment(6 * i + 3, 2), new_start, new_end);
        
            stair_aux_state_.segment(6 * i, 2) = new_start;
            stair_aux_state_.segment(6 * i + 3, 2) = new_end;
            stair_aux_state_(6 * i + 2) = stair_state_(4 * i + 2); 
            stair_aux_state_(6 * i + 5) = stair_state_(4 * i + 3);
        }
        
        // std::cout << "\033[0;36m[Stair EKF] Updated \033[0m" << std::endl; 

    }

    // Step 3: Recompute parameters
    t6 = std::chrono::high_resolution_clock::now();
    computeStaircaseParameters();
    
    t7 = std::chrono::high_resolution_clock::now();
    auto d1 = std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1); // Setting up measurement and transforming stuff
    auto d2 = std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2); // Matching (Data association) time
    auto d3 = std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3); // Model Prediction time
    auto d4 = std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4); // MISC 
    auto d5 = std::chrono::duration_cast<std::chrono::microseconds>(t6 - t5); // EKF Time
    auto d6 = std::chrono::duration_cast<std::chrono::microseconds>(t7 - t6); // Recomputing parameters
    
    process_result.success = true;
    process_result.data_association_time = d2.count();
    process_result.filter_time = d5.count();
    process_result.model_prediction_time = d3.count();
    process_result.misc_time = d1.count() + d4.count() + d6.count();

    // std::cout << staircase_params_ << std::endl;
    // std::cout << staircase_param_covariance_ << std::endl;

    return process_result;

}

/* Implementations for Client side Operations that use Staircase Estimates over Measurements */
StaircaseModel::StaircaseModel(int stair_id, const stair_utility::StaircaseEstimate& stair_estimate, stair_utility::StaircaseFilterType filter_type){
    // Initialize a staircase model from a stair estimate - This is already registered to the map frame.
    stair_id_ = stair_id;
    filter_type_ = filter_type;

    stair_count_ = stair_estimate.stair_count;
    stair_aux_state_.resize(stair_estimate.stair_count * 6);
    stair_covariance_.setZero(stair_count_ * 4, stair_count_ * 4);


    for(int i = 0; i < stair_count_; i++){
        stair_aux_state_.segment(6*i, 3) = stair_estimate.steps[i].start_p;
        stair_aux_state_.segment(6*i + 3, 3) = stair_estimate.steps[i].end_p;
        
        stair_covariance_.block(4 * i, 4 * i, 4, 4) = stair_estimate.steps[i].step_covariance;
    }
    
    computeStaircaseParameters();
}

void StaircaseModel::reInitializeStaircase(const stair_utility::StaircaseEstimate& new_staircase){
    // If a newer-estimate is received from the same robot, we don't need to merge it, we just need to update our current state
    // Merging needs to happens when estimates are coming from differnt robots, as merging happens on each robot, and we don't need to merge those on client-side
    stair_count_ = new_staircase.stair_count;
    stair_aux_state_.resize(new_staircase.stair_count * 6);

    for(int i = 0; i < stair_count_; i++){
        stair_aux_state_.segment(6*i, 3) = new_staircase.steps[i].start_p;
        stair_aux_state_.segment(6*i + 3, 3) = new_staircase.steps[i].end_p;
    }

    computeStaircaseParameters();
}

void StaircaseModel::updateStaircase(const stair_utility::StaircaseEstimate& new_staircase){

    // Step 1: Find matches between new_staircase and current state
    bool result = computeMeasurementMatches(new_staircase.stair_count, new_staircase.steps);
    if(!result)
        return;

    // Step 2: Perform averaging or EKF based on filter_type_
    if(filter_type_ == stair_utility::StaircaseFilterType::SimpleAveraging){
        applyAveragingFilter(new_staircase.stair_count, new_staircase.steps);
    }
    else{
        std::cout << "\033[0;36m[Stair EKF] Incorrect Filter Type Specified for Merging Estimates \033[0m" << std::endl;
    }

    // Step 3: Recompute parameters
    computeStaircaseParameters();
    // std::cout << staircase_params_ << std::endl;
}