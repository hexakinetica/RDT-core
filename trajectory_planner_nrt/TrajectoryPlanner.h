// TrajectoryPlanner.h
#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#pragma once

#include "DataTypes.h"             
#include "Units.h"                 
#include "KinematicSolver.h"       
#include "FrameTransformer.h"      
#include "TrajectoryInterpolator.h"
#include "Logger.h"                

#include <vector>  
#include <memory>  
#include <string>  

namespace RDT {

class TrajectoryPlanner {
public:
    TrajectoryPlanner(std::shared_ptr<KinematicSolver> solver,
                      std::shared_ptr<TrajectoryInterpolator> interpolator);

    ~TrajectoryPlanner() = default;

    TrajectoryPlanner(const TrajectoryPlanner&) = delete;
    TrajectoryPlanner& operator=(const TrajectoryPlanner&) = delete;
    TrajectoryPlanner(TrajectoryPlanner&&) = delete;
    TrajectoryPlanner& operator=(TrajectoryPlanner&&) = delete;

    void setCurrentRobotState(const TrajectoryPoint& current_robot_state);

    [[nodiscard]] bool addTargetWaypoint(const TrajectoryPoint& next_target_waypoint);

    [[nodiscard]] std::vector<TrajectoryPoint> getNextPointWindow(Seconds dt_sample, Seconds window_duration);

    [[nodiscard]] bool isCurrentSegmentDone() const;

    [[nodiscard]] bool hasError() const;
    [[nodiscard]] std::string getErrorMessage() const;
    
    void clearError();
    
private:
    [[nodiscard]] bool transformWaypointToRobotBaseFlangeTarget(const TrajectoryPoint& waypoint_in_user_frame, TrajectoryPoint& result_base_flange) const;

    bool loadSegmentForInterpolator(const TrajectoryPoint& start_point_base_flange,
                                    const TrajectoryPoint& end_point_base_flange);
    
    void setError(const std::string& message);
   
    std::shared_ptr<KinematicSolver> solver_;
    std::shared_ptr<TrajectoryInterpolator> interpolator_;

    TrajectoryPoint current_robot_state_base_flange_; 
    bool current_state_is_set_ = false;            
    bool segment_is_active_ = false;               

    bool error_flag_ = false;
    std::string error_message_;

    AxisSet last_ik_seed_joints_;

    TrajectoryPoint current_segment_user_target_waypoint_;

    static inline const std::string MODULE_NAME = "TrajPlanner";
};

} // namespace RDT
#endif // TRAJECTORYPLANNER_H