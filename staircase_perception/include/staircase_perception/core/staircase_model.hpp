#ifndef _STAIRCASE_MODEL_H_
#define _STAIRCASE_MODEL_H_

#include "staircase_perception/utils/stair_utilities.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <map>
#include <set>
#include <deque>
#include <unordered_map>

typedef Eigen::Matrix<double, 6, 1> StaircaseParameters;

typedef Eigen::VectorXd StaircaseState, StaircaseAuxState;
typedef Eigen::VectorXd StaircaseMeasurementVector;
typedef Eigen::MatrixXd StaircaseStateCovariance;

class StaircaseModel
{   
    private:
        std::map<int, int> matched_measurements_;
        std::map<int, int> matches_from_current_state_;
        std::set<int> preceding_measurements_;
        std::set<int> succeeding_measurements_;
        // std::set<int> unmatched_states_;
        std::set<int> unmatched_measurements_;

        stair_utility::StaircaseFilterType filter_type_;

        Eigen::Vector4d init_measurement_sigmas_;
        Eigen::Vector4d init_pose_sigmas_;

        Eigen::Vector4d measurement_sigmas_, process_noise_sigmas_;
        Eigen::Vector4d pose_sigmas_;
        Eigen::Matrix<double, 6, 1> staircase_model_sigmas_;

        Eigen::MatrixXd kalman_gain_, residual_covariance_;
        Eigen::VectorXd residual_vector_;
        Eigen::MatrixXd active_measurements_;

        stair_utility::StaircaseMeasurement new_staircase_registered_;

        // EKF predict and process functions
        void transformPredictedStatetoLocalFrame(const Eigen::Affine3d& transform);
        void predictStaircaseStateFromModel(const int measurement_stair_count, const std::vector<stair_utility::StairStep> &measurement_steps, const Eigen::Affine3d &transform);
        
        // Matching stairs
        bool computeMeasurementMatches(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps);
        bool computeMeasurementMatchesWithCovarianceInLocalFrame(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps, const Eigen::Affine3d& transform);

        // Averaging method
        void applyAveragingFilter(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps);

        // Maximizing method
        void applyMaximizingFilter(const int incoming_stair_count, const std::vector<stair_utility::StairStep> &new_staircase_steps);

        // Utils
        inline void projectEndpoints(float radius, float angle, const Eigen::Vector2d& start, const Eigen::Vector2d& end, Eigen::Vector2d& new_start, Eigen::Vector2d& new_end)
        {
            double s = -1.0 / tan(angle);
            double b = radius / sin(angle);
            double x = start(0);
            double y = start(1);
            new_start(0) = (s * y + x - s * b) / (pow(s, 2) + 1);
            new_start(1) = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
            x = end(0);
            y = end(1);
            new_end(0) = (s * y + x - s * b) / (pow(s, 2) + 1);
            new_end(1) = (pow(s, 2) * y + s * x + b) / (pow(s, 2) + 1);
        }

        inline std::pair<double, double> angleAndRadiusFromEndpoints(const Eigen::Vector3d& start, const Eigen::Vector3d& end)
        {
            double slope, angle;
            if (fabs(end(0) - start(0)) > 1e-9)
            {
                slope = (end(1) - start(1)) / (end(0) - start(0));
                angle = stair_utility::wrap2PI(atan(slope) + M_PI/2);
            }
            else
            {
                angle = 0.0;
            }
            
            double radius = start(0) * cos(angle) + start(1) * sin(angle);
            if (radius < 0)
            {
                radius = -radius;
                angle = stair_utility::wrap2PI(angle + M_PI);
            }

            return {radius, angle};
        }

        inline void getPreceedingRadiusAngle(const double curr_r, const double curr_theta, const Eigen::Vector2d& center, const int stair_num, std::pair<double, double>& new_r_theta, std::array<double, 3> param_jac_elements){
            
            int sign1 = 1, sign2 = 1, sign3 = 1;
            double ang = staircase_params_(3) + (stair_num * staircase_params_(4));
            if(fabs(stair_utility::wrap2PI(step_directions_[stair_num] - curr_theta)) > M_PI_2){ // Add or remove from Radius State based on staircase direction
                sign1 = -1;
            }

            double new_theta = stair_utility::wrap2PI(curr_theta - staircase_params_(4)); // Compute new angle for the stair

            double cos_delta = cos(-staircase_params_(4));
            double r_pr =  curr_r / cos_delta;
            Eigen::Vector2d r_point = {r_pr * cos(new_theta), r_pr * sin(new_theta)};  // Point on the line that is given by the (r, theta) of the state
            Eigen::Vector2d r_0_point = {curr_r * cos(curr_theta), curr_r * sin(curr_theta)};

            double len = (r_point - center).norm(); // Distance from the center of the lowermost stair to the r_point
            double len2 = (r_0_point - center).norm();
            
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

            new_r_theta.first = new_d;
            new_r_theta.second = new_theta;
            param_jac_elements[0] = -sign1 * sign2;
            param_jac_elements[1] = -sign3 * sign1 * sign2 * len * cos_delta + r_pr * tan(staircase_params_(4));
            param_jac_elements[2] = -1.0 * sign1;

        }

        inline void getSucceedingRadiusAngle(const double curr_r, const double curr_theta, const Eigen::Vector2d& center, const int stair_num, std::pair<double, double>& new_r_theta, std::array<double, 3> param_jac_elements){
            
            int sign1 = 1, sign2 = 1, sign3 = 1;
            
            double ang = staircase_params_(5) - ((stair_count_ - 1 - stair_num) * staircase_params_(4));
            if(fabs(stair_utility::wrap2PI(step_directions_[stair_num] - curr_theta)) > M_PI_2){ // Add or remove from Radius State based on staircase direction
                sign1 = -1;
            }

            double new_theta = stair_utility::wrap2PI(curr_theta + staircase_params_(4)); // Compute new angle for the stair

            double cos_delta = cos(staircase_params_(4));
            double r_pr =  curr_r / cos_delta;
            Eigen::Vector2d r_point = {r_pr * cos(new_theta), r_pr * sin(new_theta)};  // Point on the line that is given by the (r, theta) of the state
            Eigen::Vector2d r_0_point = {curr_r * cos(curr_theta), curr_r * sin(curr_theta)};

            double len = (r_point - center).norm(); // Distance from the center of the lowermost stair to the r_point
            double len2 = (r_0_point - center).norm();
            
            if((len > len2 && staircase_params_(4) < 0) || (len < len2 && staircase_params_(4) > 0)){
                // std::cout << "len1 :" << len << " len2: " << len2 << " delt:" << staircase_params_(4) <<  std::endl;
                // std::cout << "Sign3 switched directions" << std::endl;
                sign3 = -1;
            }

            double new_d = r_pr + (sign1 * (staircase_params_(1) - (len * sin(sign1 * sign3 * staircase_params_(4))))); // Compute new radius for the stair
            
            if(new_d < 0){
                sign2 = -1;
                new_d = -new_d;
                new_theta = stair_utility::wrap2PI(new_theta + M_PI);
            }

            new_r_theta.first = new_d;
            new_r_theta.second = new_theta;
            param_jac_elements[0] = sign1 * sign2;
            param_jac_elements[1] = sign3 * sign1 * sign2 * len * cos_delta + r_pr * tan(staircase_params_(4));
            param_jac_elements[2] = sign1;
            
        }

    public:
        int stair_id_;
        int stair_count_;
        // Number of world-frame observations associated to this staircase (starts at 1 on creation).
        int times_observed_ = 1;

        StaircaseParameters staircase_params_; 
        stair_utility::StaircaseInfo staircase_info_;

        StaircaseState stair_state_, stair_aux_state_; //Staircase State uses r, theta, z. Aux state holds endpoints of the staircase
        StaircaseStateCovariance stair_covariance_;
        std::vector<double> step_directions_;

        StaircaseState predicted_state_, predicted_aux_state_;
        StaircaseStateCovariance predicted_covariance_, predicted_local_covariance_;
        StaircaseMeasurementVector input_measurement_vector_, predicted_measurement_vector_, registered_input_aux_measurement_, registered_input_measurement_;

        StaircaseModel(){};
        ~StaircaseModel(){};
        StaircaseModel(int stair_id_, const stair_utility::StaircaseMeasurement& stair_measurement, stair_utility::StaircaseFilterType filter_type, const stair_utility::constantWidthEKFParams& params = stair_utility::constantWidthEKFParams());
        StaircaseModel(int stair_id_, const stair_utility::StaircaseEstimate& stair_estimate, stair_utility::StaircaseFilterType filter_type);

        const double getStairHeight(){
            return staircase_params_(0);
        }

        const double getStairDepth(){
            return staircase_params_(1);
        }

        const double getStairWidth(){
            return staircase_params_(2);
        }

        const double getStairStartAngle(){
            return staircase_params_(3);
        }

        const double getStairEndAngle(){
            return staircase_params_(5);
        }

        const double getStairDeltaAngle(){
            return staircase_params_(4);
        }
        
        const int getStairCount(){
            return stair_count_;
        }

        void computeStaircaseParameters();

        // Basic Methods for staircase updates on robot (as it gets measurements)
        stair_utility::StaircaseProcessingResult updateStaircase(const stair_utility::StaircaseMeasurement& new_staircase);

        // Basic Methods for staircase updates on client-side (as it gets estimates)
        void reInitializeStaircase(const stair_utility::StaircaseEstimate& new_staircase);
        void updateStaircase(const stair_utility::StaircaseEstimate& new_staircase);

};


#endif