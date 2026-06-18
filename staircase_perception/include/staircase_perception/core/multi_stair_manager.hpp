#ifndef _MULTI_STAIR_MANAGER_H_
#define _MULTI_STAIR_MANAGER_H_

#include "staircase_perception/utils/stair_utilities.hpp"
#include "staircase_perception/core/staircase_model.hpp"
#include <unordered_map>
#include <unordered_set>


class BaseStaircaseManager{
    public:
        BaseStaircaseManager(){};
        ~BaseStaircaseManager(){};

        stair_utility::StairManagerParams stair_manager_params_;
        int number_of_stairs_;

        bool areLinesIntersecting(const Eigen::Vector3d &pointA, const Eigen::Vector3d &pointB, const Eigen::Vector3d &pointC, const Eigen::Vector3d &pointD);
        int checkStaircaseSimilarity(const stair_utility::StaircaseInfo& curr_stairase_info, const stair_utility::StaircaseInfo& new_staircase_info); 
        
    protected:
        int getNewId();

        void getStairInfoFromCorners(const Eigen::Vector3d &begin_start, const Eigen::Vector3d &begin_end, const Eigen::Vector3d &final_start, const Eigen::Vector3d &final_end, stair_utility::StaircaseInfo& stair_info);

        std::unordered_set<int> ids_;
};


class SingleRobotStairManager: public BaseStaircaseManager{
    public:
        SingleRobotStairManager(){};
        ~SingleRobotStairManager(){};
        SingleRobotStairManager(const stair_utility::StairManagerParams& params);

        // is_confirmed is set true once the staircase has been observed at least
        // min_detections_to_confirm times in the world frame (filters viewpoint-dependent false positives).
        int addNewDetectedStaircase(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseEstimate &estimate, bool& is_confirmed);

        stair_utility::StaircaseProcessingResult time_results_;
    private:

        void computeStaircaseInfo(const stair_utility::StaircaseMeasurement& new_staircase, stair_utility::StaircaseInfo& stair_info);

        stair_utility::StaircaseFilterType filter_type_;
        int min_detections_to_confirm_;
        std::unordered_map<int, std::shared_ptr<StaircaseModel>> staircase_database_;

};

#endif