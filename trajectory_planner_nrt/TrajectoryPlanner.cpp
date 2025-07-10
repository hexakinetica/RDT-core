// TrajectoryPlanner.cpp
#include "TrajectoryPlanner.h"
#include <stdexcept> // For std::invalid_argument, std::runtime_error
#include <cmath>     // For std::abs

namespace RDT {

// For convenience with literals
using namespace RDT::literals;

TrajectoryPlanner::TrajectoryPlanner(std::shared_ptr<KinematicSolver> solver,
                                     std::shared_ptr<TrajectoryInterpolator> interpolator)
    : solver_(std::move(solver)),
      interpolator_(std::move(interpolator)) {
    if (!solver_) {
        LOG_CRITICAL_F(MODULE_NAME, "KinematicSolver cannot be null.");
        throw std::invalid_argument("TrajectoryPlanner: KinematicSolver cannot be null.");
    }
    if (!interpolator_) {
        LOG_CRITICAL_F(MODULE_NAME, "TrajectoryInterpolator cannot be null.");
        throw std::invalid_argument("TrajectoryPlanner: TrajectoryInterpolator cannot be null.");
    }
    LOG_INFO(MODULE_NAME, "TrajectoryPlanner initialized.");
}

void TrajectoryPlanner::setCurrentRobotState(const TrajectoryPoint& current_robot_state) {
    clearError();
    LOG_INFO(MODULE_NAME, "Setting current robot state.");

    current_robot_state_base_flange_ = current_robot_state; // Copy initial state
    // Ensure tool/base in this internal state are default, as it represents flange in base.
    current_robot_state_base_flange_.header.tool = ToolFrame(); // Default tool (identity on flange)
    current_robot_state_base_flange_.header.base = BaseFrame(); // Default base (identity to robot base)

    if (current_robot_state_base_flange_.command.joint_target.size() != ROBOT_AXES_COUNT) {
        setError("setCurrentRobotState: Initial state must have valid joint_target size.");
        current_state_is_set_ = false;
        return;
    }

    // Ensure Cartesian pose is consistent with joint pose (FK for flange)
    try {
        current_robot_state_base_flange_.command.cartesian_target = solver_->solveFK(current_robot_state_base_flange_.command.joint_target);
    } catch (const std::exception& e) {
        setError("setCurrentRobotState: FK failed for initial joint positions: " + std::string(e.what()));
        current_state_is_set_ = false;
        return;
    }

    last_ik_seed_joints_ = current_robot_state_base_flange_.command.joint_target;
    current_state_is_set_ = true;
    segment_is_active_ = false; // New state invalidates any active segment
    LOG_INFO_F(MODULE_NAME, "Current robot state set. Flange Base: %s, Joints (deg): %s",
               current_robot_state_base_flange_.command.cartesian_target.toDescriptiveString().c_str(),
               current_robot_state_base_flange_.command.joint_target.toJointPoseString().c_str());
}

TrajectoryPoint TrajectoryPlanner::transformWaypointToRobotBaseFlangeTarget(const TrajectoryPoint& waypoint_in_user_frame) {
    LOG_DEBUG_F(MODULE_NAME, "Transforming waypoint. User Base: '%s', User Tool: '%s'",
                waypoint_in_user_frame.header.base.name.c_str(),
                waypoint_in_user_frame.header.tool.name.c_str());

    TrajectoryPoint target_base_flange = waypoint_in_user_frame; // Start by copying all header/command info

    // The target waypoint's pose_cart is TCP in UserFrame (defined by waypoint_in_user_frame.header.base)
    // T_userBase_tcp = waypoint_in_user_frame.command.pose_cart
    // T_robotBase_userBase = waypoint_in_user_frame.header.base.transform
    // T_flange_tcp = waypoint_in_user_frame.header.tool.transform

    // 1. Calculate TCP pose in Robot Base Frame:
    // T_robotBase_tcp = T_robotBase_userBase * P_userBase_tcp (where P_userBase_tcp is the command.pose_cart)
    CartPose tcp_pose_in_robot_base = FrameTransformer::combineTransforms(
        waypoint_in_user_frame.header.base.transform, // This is T_robotBase_userFrame
        waypoint_in_user_frame.command.cartesian_target      // This is P_userFrame_TCP
    );
    LOG_DEBUG_F(MODULE_NAME, "  TCP in Robot Base (after base transform): %s", tcp_pose_in_robot_base.toDescriptiveString().c_str());


    // 2. Calculate Flange pose in Robot Base Frame from TCP pose in Robot Base Frame:
    // T_robotBase_flange = T_robotBase_tcp * inv(T_flange_tcp)
    CartPose flange_pose_in_robot_base = FrameTransformer::calculateFlangeInWorld( // "World" here means Robot Base
        tcp_pose_in_robot_base,
        waypoint_in_user_frame.header.tool.transform // This is T_flange_tcp
    );
    LOG_DEBUG_F(MODULE_NAME, "  Flange in Robot Base (after tool inverse): %s", flange_pose_in_robot_base.toDescriptiveString().c_str());

    target_base_flange.command.cartesian_target = flange_pose_in_robot_base;

    // After transformation, the point is now relative to robot base with zero tool.
    target_base_flange.header.base = BaseFrame(); // Identity base
    target_base_flange.header.tool = ToolFrame(); // Identity tool (represents flange)

    // For JOINT/PTP motion, the original joint_target is preserved.
    // The cartesian_target just calculated is for information or for LIN target interpretation.
    if (waypoint_in_user_frame.header.motion_type == MotionType::JOINT ||
        waypoint_in_user_frame.header.motion_type == MotionType::PTP) {
        // Keep original joint target, cartesian_target is now flange pose from FK of these joints
        target_base_flange.command.joint_target = waypoint_in_user_frame.command.joint_target;
        try {
            target_base_flange.command.cartesian_target = solver_->solveFK(target_base_flange.command.joint_target);
             LOG_DEBUG_F(MODULE_NAME, "  JOINT/PTP target: Updated flange cartesian from FK: %s", target_base_flange.command.cartesian_target.toDescriptiveString().c_str());
        } catch (const std::exception& e) {
            LOG_WARN_F(MODULE_NAME, "  JOINT/PTP target: FK failed for original joint target after transform: %s. Cartesian part might be inconsistent.", e.what());
            // Cartesian part will hold the transformed TCP-as-flange, which is not ideal for JOINT.
            // This indicates an issue if the user provided inconsistent joint/cartesian for a JOINT target in user frame.
        }
    } else if (waypoint_in_user_frame.header.motion_type == MotionType::LIN) {
        // For LIN, the cartesian_target (flange in base) is primary.
        // The joint_target from user input is irrelevant; it will be solved by IK.
        // We can clear it or leave it; it won't be used by interpolator for LIN profile math.
        target_base_flange.command.joint_target = AxisSet{}; // Clear to avoid confusion
        LOG_DEBUG(MODULE_NAME, "  LIN target: Joint target cleared, cartesian flange target is primary.");
    }
    return target_base_flange;
}


bool TrajectoryPlanner::addTargetWaypoint(const TrajectoryPoint& next_target_waypoint) {
    clearError();
    if (!current_state_is_set_) {
        setError("Cannot add target: current robot state not set.");
        return false;
    }
    if (segment_is_active_ && !interpolator_->isIdle()) {
        LOG_WARN(MODULE_NAME, "Cannot add new target: current segment interpolation is still active.");
        return false; // Not an error, planner is busy
    }
    LOG_INFO(MODULE_NAME, "Adding new target waypoint.");

    current_segment_user_target_waypoint_ = next_target_waypoint; // Cache user target for metadata

    TrajectoryPoint target_for_flange_in_base;
    try {
        target_for_flange_in_base = transformWaypointToRobotBaseFlangeTarget(next_target_waypoint);
    } catch (const std::exception& e) {
        setError("Failed to transform target waypoint to robot base: " + std::string(e.what()));
        return false;
    }

    // For LIN movements, we need an initial IK solution for the target flange pose
    // to ensure it's reachable and to have a joint target for the interpolator if it needs one
    // (though TrapProfileLIN primarily uses Cartesian).
    // For JOINT/PTP, joint_target is already set in target_for_flange_in_base.
    if (target_for_flange_in_base.header.motion_type == MotionType::LIN) {
        LOG_DEBUG_F(MODULE_NAME, "LIN target: performing IK for flange pose: %s", target_for_flange_in_base.command.cartesian_target.toDescriptiveString().c_str());
        std::optional<AxisSet> ik_for_lin_target = solver_->solveIK(
            target_for_flange_in_base.command.cartesian_target, // Target is flange pose
            last_ik_seed_joints_ // Seed with last known good joints
        );
        if (ik_for_lin_target) {
            target_for_flange_in_base.command.joint_target = *ik_for_lin_target;
            LOG_DEBUG_F(MODULE_NAME, "  IK for LIN target success: %s", ik_for_lin_target->toJointPoseString().c_str());
        } else {
            setError("IK failed for LIN target's Cartesian pose (flange in base). Target may be unreachable.");
            return false;
        }
    }
    // For JOINT/PTP, target_for_flange_in_base.command.joint_target comes from user,
    // and target_for_flange_in_base.command.cartesian_target is FK(joint_target).

    // Now, current_robot_state_base_flange_ is the start,
    // and target_for_flange_in_base is the end. Both are flange poses in base frame.
    return loadSegmentForInterpolator(current_robot_state_base_flange_, target_for_flange_in_base);
}

bool TrajectoryPlanner::loadSegmentForInterpolator(const TrajectoryPoint& start_point_base_flange,
                                                 const TrajectoryPoint& end_point_base_flange) {
    LOG_DEBUG_F(MODULE_NAME, "Loading segment into interpolator. Start Flange (Cmd): %s", start_point_base_flange.command.cartesian_target.toDescriptiveString().c_str());
    LOG_DEBUG_F(MODULE_NAME, "End Flange (Cmd): %s", end_point_base_flange.command.cartesian_target.toDescriptiveString().c_str());
    LOG_DEBUG_F(MODULE_NAME, "End MotionType for interpolator: %d", static_cast<int>(end_point_base_flange.header.motion_type));


    // The interpolator expects TrajectoryPoints where the .command part contains the
    // start and end poses for its profile math (either joint_target or cartesian_target).
    // It also uses .header.motion_type from the *target* to select profile type,
    // and .command.speed_ratio etc. from the *target* for profile parameters.
    try {
        interpolator_->loadSegment(start_point_base_flange, end_point_base_flange);
        segment_is_active_ = true;
        // last_ik_seed_joints_ should be updated after segment completion, or before next IK.
        // For now, it's updated from current_robot_state_base_flange_ in addTargetWaypoint.
        return true;
    } catch (const std::exception& e) {
        setError("Failed to load segment into interpolator: " + std::string(e.what()));
        segment_is_active_ = false;
        return false;
    }
}


std::vector<TrajectoryPoint> TrajectoryPlanner::getNextPointWindow(Seconds dt_sample, Seconds window_duration) {
    std::vector<TrajectoryPoint> window_points;
    if (!segment_is_active_ || error_flag_) {
        if (error_flag_) LOG_WARN_F(MODULE_NAME, "getNextPointWindow called while in error state: %s", error_message_.c_str());
        else if (!segment_is_active_) LOG_DEBUG(MODULE_NAME, "getNextPointWindow called but no segment is active.");
        return window_points; // Empty if no active segment or error
    }
    if (dt_sample <= 0.0_s || window_duration <= 0.0_s) {
        setError("dt_sample and window_duration must be positive for getNextPointWindow.");
        return window_points;
    }

    LOG_DEBUG_F(MODULE_NAME, "Generating window: dt=%.3fms, win_dur=%.3fms. Interpolator idle: %s",
        dt_sample.value()*1000.0, window_duration.value()*1000.0, interpolator_->isIdle()?"Y":"N");

    Seconds accumulated_time_in_window = 0.0_s;

    while (accumulated_time_in_window < window_duration && !interpolator_->isIdle() && !error_flag_) {
        TrajectoryPoint interpolated_flange_point; // Point from interpolator (flange in base)
        try {
            interpolated_flange_point = interpolator_->nextPoint(dt_sample);
        } catch (const std::exception& e) {
            setError("Error from interpolator_->nextPoint(): " + std::string(e.what()));
            break; // Exit window generation on interpolator error
        }

        // `interpolated_flange_point` has .command.joint_target (for JOINT/PTP profile)
        // or .command.cartesian_target (for LIN profile, representing flange)
        // and .header metadata from the original user target (tool, base, original motion type).

        TrajectoryPoint rt_point_for_mm; // This is what we send to MotionManager
        rt_point_for_mm.header = current_segment_user_target_waypoint_.header; // Original user metadata (tool, base, etc.)
        // Overwrite motion_type with the one from interpolated point (which reflects profile)
        // although, original motion type from user target is usually more relevant for higher levels.
        // Let's keep the original user target's motion type, as the interpolated point is a step *towards* it.
        // rt_point_for_mm.header.motion_type = interpolated_flange_point.header.motion_type;
        rt_point_for_mm.header.segment_duration = interpolator_->getCurrentProfileDuration(); // Total duration of this segment
        rt_point_for_mm.header.is_target_reached_for_this_point = interpolated_flange_point.header.is_target_reached_for_this_point; // From interpolator cache

        if (interpolated_flange_point.header.motion_type == MotionType::LIN) {
            // Interpolator gave us flange_pose_in_base (Cartesian)
            // We need joint_target for MotionManager. Solve IK.
            CartPose target_flange_for_ik = interpolated_flange_point.command.cartesian_target;
            LOG_DEBUG_F(MODULE_NAME, "  LIN point: Target flange for IK: %s", target_flange_for_ik.toDescriptiveString().c_str());
            std::optional<AxisSet> ik_result = solver_->solveIK(target_flange_for_ik, last_ik_seed_joints_);
            if (ik_result) {
                rt_point_for_mm.command.joint_target = *ik_result;
                rt_point_for_mm.command.cartesian_target = target_flange_for_ik; // Store the flange pose from interpolator
                last_ik_seed_joints_ = *ik_result;
                rt_point_for_mm.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD; // MM expects joints
            } else {
                setError("IK FAILED for interpolated LIN point (flange): " + target_flange_for_ik.toDescriptiveString());
                break; // Stop generating this window
            }
        } else { // JOINT or PTP (interpolator gives joint_target)
            rt_point_for_mm.command.joint_target = interpolated_flange_point.command.joint_target;
            // For MotionManager, provide consistent cartesian_target (flange pose from FK)
            try {
                rt_point_for_mm.command.cartesian_target = solver_->solveFK(rt_point_for_mm.command.joint_target);
            } catch (const std::exception& e) {
                LOG_WARN_F(MODULE_NAME, "  FK failed for interpolated JOINT point: %s. Cartesian part in RT point may be stale.", e.what());
                // Keep potentially stale cartesian_target from interpolator or default
                rt_point_for_mm.command.cartesian_target = interpolated_flange_point.command.cartesian_target;
            }
            last_ik_seed_joints_ = rt_point_for_mm.command.joint_target;
            rt_point_for_mm.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD; // MM expects joints
        }
        
        // The points sent to MM are now always flange in base, with joint commands
        // So, tool and base in the RT point header should be identity.
        rt_point_for_mm.header.tool = ToolFrame(); 
        rt_point_for_mm.header.base = BaseFrame();

        window_points.push_back(rt_point_for_mm);
        accumulated_time_in_window += dt_sample;
    } // end while

    // Update current_robot_state_base_flange_ to the last successfully generated point in the window
    if (!window_points.empty() && !error_flag_) {
        // The .command part of the last point in window_points contains the
        // latest calculated joint_target and corresponding cartesian_target (flange).
        current_robot_state_base_flange_.command = window_points.back().command;
        // Keep header from current_robot_state_base_flange_ (which is for flange in base)
    } else if (error_flag_) {
        // If an error occurred, current_robot_state_base_flange_ remains at the start of this problematic window attempt.
        // The IK seed (last_ik_seed_joints_) also remains from before the error.
        LOG_WARN(MODULE_NAME, "Window generation ended with error. Robot state not advanced past last good point.");
    }


    if (interpolator_->isIdle()) {
        segment_is_active_ = false;
        LOG_INFO_F(MODULE_NAME, "Current segment interpolation complete. Total points in window: %zu", window_points.size());
        // Ensure current_robot_state_base_flange_ is exactly the target of the segment if successful
        if(!error_flag_){
            TrajectoryPoint final_target_flange_base;
            try {
                // Re-transform original user target to ensure precision, as interpolator's final point might have small diffs
                final_target_flange_base = transformWaypointToRobotBaseFlangeTarget(current_segment_user_target_waypoint_);
                 if (current_segment_user_target_waypoint_.header.motion_type == MotionType::LIN) {
                    auto ik_final = solver_->solveIK(final_target_flange_base.command.cartesian_target, last_ik_seed_joints_);
                    if(ik_final) final_target_flange_base.command.joint_target = *ik_final;
                    else { LOG_WARN(MODULE_NAME, "IK failed for final LIN target on segment completion. Joint state might be approximate.");}
                 } else { // JOINT/PTP
                    // joint_target is already correct from user, cartesian_target is from FK in transformWaypoint...
                 }
            } catch(const std::exception& e) {
                LOG_ERROR_F(MODULE_NAME, "Error re-transforming final segment target: %s. Using last interpolated point as current state.", e.what());
                // current_robot_state_base_flange_ already holds the last good point from the window.
            }
            current_robot_state_base_flange_.command = final_target_flange_base.command;
            last_ik_seed_joints_ = current_robot_state_base_flange_.command.joint_target;
            LOG_INFO_F(MODULE_NAME, "Robot state updated to precise segment end. Flange: %s", current_robot_state_base_flange_.command.cartesian_target.toDescriptiveString().c_str());
        }
    }
    return window_points;
}


bool TrajectoryPlanner::isCurrentSegmentDone() const {
    if (error_flag_) return true; // If error, consider segment "done" in a bad way
    return !segment_is_active_ || (interpolator_ && interpolator_->isIdle());
}

bool TrajectoryPlanner::hasError() const {
    return error_flag_;
}

std::string TrajectoryPlanner::getErrorMessage() const {
    return error_message_;
}

void TrajectoryPlanner::setError(const std::string& message) {
    if (!error_flag_) { // Log and set error only once to keep the root cause
        error_message_ = message;
        error_flag_ = true;
        LOG_ERROR_F(MODULE_NAME, "Error set: %s", message.c_str());
    } else {
        LOG_WARN_F(MODULE_NAME, "Additional error context (ignored as error already set): %s", message.c_str());
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