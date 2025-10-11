// TrajectoryPlanner.cpp
#include "TrajectoryPlanner.h"
//#include "FrameTransformer.h"
#include <stdexcept> // For std::invalid_argument in constructor
#include <cmath>     // For std::abs

namespace RDT {

using namespace RDT::literals;

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<KinematicSolver> solver,
                                     std::shared_ptr<TrajectoryInterpolator> interpolator)
    : solver_(std::move(solver)),
      interpolator_(std::move(interpolator)) {
    if (!solver_) {
        throw std::invalid_argument("TrajectoryPlanner: KinematicSolver cannot be null.");
    }
    if (!interpolator_) {
        throw std::invalid_argument("TrajectoryPlanner: TrajectoryInterpolator cannot be null.");
    }
    LOG_INFO(MODULE_NAME, "TrajectoryPlanner initialized.");
}

void TrajectoryPlanner::setCurrentRobotState(const TrajectoryPoint& current_robot_state) {
    clearError();
    LOG_INFO(MODULE_NAME, "Setting current robot state.");

    current_robot_state_base_flange_ = current_robot_state;
    current_robot_state_base_flange_.header.tool = ToolFrame();
    current_robot_state_base_flange_.header.base = BaseFrame();

    if (current_robot_state_base_flange_.command.joint_target.size() != ROBOT_AXES_COUNT) {
        setError("setCurrentRobotState: Initial state must have valid joint_target size.");
        current_state_is_set_ = false;
        return;
    }

    // MODIFIED: Replaced try-catch with bool check for solveFK
    if (!solver_->solveFK(current_robot_state_base_flange_.command.joint_target, 
                          current_robot_state_base_flange_.command.cartesian_target)) {
        setError("setCurrentRobotState: FK failed for initial joint positions.");
        current_state_is_set_ = false;
        return;
    }

    last_ik_seed_joints_ = current_robot_state_base_flange_.command.joint_target;
    current_state_is_set_ = true;
    segment_is_active_ = false;
    LOG_INFO_F(MODULE_NAME, "Current robot state set. Flange Base: %s, Joints (deg): %s",
               current_robot_state_base_flange_.command.cartesian_target.toDescriptiveString().c_str(),
               current_robot_state_base_flange_.command.joint_target.toJointPoseString().c_str());
}

// MODIFIED: Changed return type to bool and added result as an output parameter
bool TrajectoryPlanner::transformWaypointToRobotBaseFlangeTarget(const TrajectoryPoint& waypoint_in_user_frame, TrajectoryPoint& result_base_flange) const {
    LOG_DEBUG_F(MODULE_NAME, "Transforming waypoint. User Base: '%s', User Tool: '%s'",
                waypoint_in_user_frame.header.base.name.c_str(),
                waypoint_in_user_frame.header.tool.name.c_str());

    result_base_flange = waypoint_in_user_frame;

    CartPose tcp_pose_in_robot_base = FrameTransformer::combineTransforms(
        waypoint_in_user_frame.header.base.transform,
        waypoint_in_user_frame.command.cartesian_target
    );
    LOG_DEBUG_F(MODULE_NAME, "  TCP in Robot Base (after base transform): %s", tcp_pose_in_robot_base.toDescriptiveString().c_str());

    CartPose flange_pose_in_robot_base = FrameTransformer::calculateFlangeInWorld(
        tcp_pose_in_robot_base,
        waypoint_in_user_frame.header.tool.transform
    );
    LOG_DEBUG_F(MODULE_NAME, "  Flange in Robot Base (after tool inverse): %s", flange_pose_in_robot_base.toDescriptiveString().c_str());

    result_base_flange.command.cartesian_target = flange_pose_in_robot_base;
    result_base_flange.header.base = BaseFrame();
    result_base_flange.header.tool = ToolFrame();

    if (waypoint_in_user_frame.header.motion_type == MotionType::JOINT ||
        waypoint_in_user_frame.header.motion_type == MotionType::PTP) {
        
        result_base_flange.command.joint_target = waypoint_in_user_frame.command.joint_target;
        
        // MODIFIED: Replaced try-catch with bool check for solveFK
        if (!solver_->solveFK(result_base_flange.command.joint_target, result_base_flange.command.cartesian_target)) {
            LOG_WARN_F(MODULE_NAME, "  JOINT/PTP target: FK failed for original joint target after transform. Cartesian part might be inconsistent.");
            // Return false to indicate a problem, even if it's just a warning.
            return false;
        }
        LOG_DEBUG_F(MODULE_NAME, "  JOINT/PTP target: Updated flange cartesian from FK: %s", result_base_flange.command.cartesian_target.toDescriptiveString().c_str());
        
    } else if (waypoint_in_user_frame.header.motion_type == MotionType::LIN) {
        result_base_flange.command.joint_target = AxisSet{};
        LOG_DEBUG(MODULE_NAME, "  LIN target: Joint target cleared, cartesian flange target is primary.");
    }
    return true;
}


bool TrajectoryPlanner::addTargetWaypoint(const TrajectoryPoint& next_target_waypoint) {
    clearError();
    if (!current_state_is_set_) {
        setError("Cannot add target: current robot state not set.");
        return false;
    }
    if (segment_is_active_ && !interpolator_->isIdle()) {
        LOG_WARN(MODULE_NAME, "Cannot add new target: current segment interpolation is still active.");
        return false;
    }
    LOG_INFO(MODULE_NAME, "Adding new target waypoint.");

    current_segment_user_target_waypoint_ = next_target_waypoint;

    TrajectoryPoint target_for_flange_in_base;
    // MODIFIED: Using new bool return type for transform function
    if (!transformWaypointToRobotBaseFlangeTarget(next_target_waypoint, target_for_flange_in_base)) {
        setError("Failed to transform target waypoint to robot base.");
        return false;
    }

    if (target_for_flange_in_base.header.motion_type == MotionType::LIN) {
        LOG_DEBUG_F(MODULE_NAME, "LIN target: performing IK for flange pose: %s", target_for_flange_in_base.command.cartesian_target.toDescriptiveString().c_str());
        
    //     // MODIFIED: Using new IKResult return type
    //     IKResult ik_result = solver_->solveIK(
    //         target_for_flange_in_base.command.cartesian_target,
    //         last_ik_seed_joints_
    //     );
        
    //     if (ik_result.status == IKStatus::Failed) {
    //         setError("IK failed for LIN target's Cartesian pose. Target may be unreachable.");
    //         return false;
    //     }
        
    //     target_for_flange_in_base.command.joint_target = ik_result.solution;

    //     if (ik_result.status == IKStatus::Approximate) {
    //         LOG_WARN(MODULE_NAME, "  IK for LIN target is APPROXIMATE. Trajectory may have small deviations.");
    //     } else {
    //         LOG_DEBUG_F(MODULE_NAME, "  IK for LIN target success: %s", ik_result.solution.toJointPoseString().c_str());
    //     }
     }
    
    return loadSegmentForInterpolator(current_robot_state_base_flange_, target_for_flange_in_base);
}

bool TrajectoryPlanner::loadSegmentForInterpolator(const TrajectoryPoint& start_point_base_flange,
                                                 const TrajectoryPoint& end_point_base_flange) {
    LOG_DEBUG_F(MODULE_NAME, "Loading segment into interpolator. Start Flange (Cmd): %s", start_point_base_flange.command.cartesian_target.toDescriptiveString().c_str());
    LOG_DEBUG_F(MODULE_NAME, "End Flange (Cmd): %s", end_point_base_flange.command.cartesian_target.toDescriptiveString().c_str());
    
    // MODIFIED: Replaced try-catch with error flag check. loadSegment now shouldn't throw.
    interpolator_->loadSegment(start_point_base_flange, end_point_base_flange);
    // Assuming loadSegment is modified to not throw std::exception but set an internal error state if needed.
    // If interpolator can fail, it should also return a bool or have a hasError() method.
    // For now, we assume it succeeds if no exception is thrown.
    
    segment_is_active_ = true;
    return true;
}


std::vector<TrajectoryPoint> TrajectoryPlanner::getNextPointWindow(Seconds dt_sample, Seconds window_duration) {
    std::vector<TrajectoryPoint> window_points;
    if (!segment_is_active_ || error_flag_) {
        return window_points;
    }
    if (dt_sample <= 0.0_s || window_duration <= 0.0_s) {
        setError("dt_sample and window_duration must be positive.");
        return window_points;
    }

    LOG_DEBUG_F(MODULE_NAME, "Generating window: dt=%.3fms, win_dur=%.3fms. Interpolator idle: %s",
        dt_sample.value()*1000.0, window_duration.value()*1000.0, interpolator_->isIdle()?"Y":"N");

    Seconds accumulated_time_in_window = 0.0_s;

    while (accumulated_time_in_window < window_duration && !interpolator_->isIdle() && !error_flag_) {
        TrajectoryPoint interpolated_flange_point = interpolator_->nextPoint(dt_sample);
        
        TrajectoryPoint rt_point_for_mm;
        rt_point_for_mm.header = current_segment_user_target_waypoint_.header;
        rt_point_for_mm.header.segment_duration = interpolator_->getCurrentProfileDuration();
        rt_point_for_mm.header.is_target_reached_for_this_point = interpolated_flange_point.header.is_target_reached_for_this_point;

        if (interpolated_flange_point.header.motion_type == MotionType::LIN) {
            CartPose target_flange_for_ik = interpolated_flange_point.command.cartesian_target;
            
            // MODIFIED: Using new IKResult return type
            IKResult ik_result = solver_->solveIK(target_flange_for_ik, last_ik_seed_joints_);
            
            if (ik_result.status == IKStatus::Failed) {
                setError("IK FAILED for interpolated LIN point (flange): " + target_flange_for_ik.toDescriptiveString());
                break;
            }
            
            if (ik_result.status == IKStatus::Approximate) {
                LOG_WARN_F(MODULE_NAME, "  Approximate IK solution for interpolated LIN point at t=%.3fs", interpolator_->getCurrentTimeInProfile().value());
            }

            rt_point_for_mm.command.joint_target = ik_result.solution;
            // For MotionManager, provide consistent cartesian_target (flange pose from FK)
            if (!solver_->solveFK(rt_point_for_mm.command.joint_target, rt_point_for_mm.command.cartesian_target)) {
                 LOG_WARN_F(MODULE_NAME, "  FK failed for interpolated LIN point. Cartesian part in RT point may be stale.");
            }
            last_ik_seed_joints_ = ik_result.solution;

        } else { // JOINT or PTP
            rt_point_for_mm.command.joint_target = interpolated_flange_point.command.joint_target;
            if (!solver_->solveFK(rt_point_for_mm.command.joint_target, rt_point_for_mm.command.cartesian_target)) {
                LOG_WARN_F(MODULE_NAME, "  FK failed for interpolated JOINT point. Cartesian part in RT point may be stale.");
            }
            last_ik_seed_joints_ = rt_point_for_mm.command.joint_target;
        }
        
        rt_point_for_mm.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
        rt_point_for_mm.header.tool = ToolFrame(); 
        rt_point_for_mm.header.base = BaseFrame();

        window_points.push_back(rt_point_for_mm);
        accumulated_time_in_window += dt_sample;
    }

    if (!window_points.empty() && !error_flag_) {
        current_robot_state_base_flange_.command = window_points.back().command;
    }

    if (interpolator_->isIdle()) {
        segment_is_active_ = false;
        LOG_INFO_F(MODULE_NAME, "Current segment interpolation complete. Total points in window: %zu", window_points.size());
        if(!error_flag_){
            TrajectoryPoint final_target_flange_base;
            if (transformWaypointToRobotBaseFlangeTarget(current_segment_user_target_waypoint_, final_target_flange_base)) {
                 if (current_segment_user_target_waypoint_.header.motion_type == MotionType::LIN) {
                    IKResult ik_final = solver_->solveIK(final_target_flange_base.command.cartesian_target, last_ik_seed_joints_);
                    if(ik_final.status != IKStatus::Failed) {
                        final_target_flange_base.command.joint_target = ik_final.solution;
                    } else { 
                        LOG_WARN(MODULE_NAME, "IK failed for final LIN target on segment completion. Joint state might be approximate.");
                    }
                 }
                current_robot_state_base_flange_.command = final_target_flange_base.command;
                last_ik_seed_joints_ = current_robot_state_base_flange_.command.joint_target;
                LOG_INFO_F(MODULE_NAME, "Robot state updated to precise segment end. Flange: %s", current_robot_state_base_flange_.command.cartesian_target.toDescriptiveString().c_str());
            } else {
                 LOG_ERROR_F(MODULE_NAME, "Error re-transforming final segment target. Using last interpolated point as current state.");
            }
        }
    }
    return window_points;
}

bool TrajectoryPlanner::isCurrentSegmentDone() const {
    if (error_flag_) return true;
    return !segment_is_active_ || (interpolator_ && interpolator_->isIdle());
}

bool TrajectoryPlanner::hasError() const {
    return error_flag_;
}

std::string TrajectoryPlanner::getErrorMessage() const {
    return error_message_;
}

void TrajectoryPlanner::setError(const std::string& message) {
    if (!error_flag_) {
        error_message_ = message;
        error_flag_ = true;
        LOG_ERROR_F(MODULE_NAME, "Error set: %s", message.c_str());
    }
}

void TrajectoryPlanner::clearError() {
    if (error_flag_) {
        LOG_INFO_F(MODULE_NAME, "Clearing error state. Previous error: %s", error_message_.c_str());
    }
    error_flag_ = false;
    error_message_.clear();
}

} // namespace RDT