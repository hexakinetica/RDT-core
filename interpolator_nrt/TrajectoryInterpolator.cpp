// TrajectoryInterpolator.cpp
#include "TrajectoryInterpolator.h"
#include <cmath> // For std::abs, std::sqrt
#include <format> // For std::format

namespace RDT {

using namespace RDT::literals;

TrajectoryInterpolator::TrajectoryInterpolator()
    : time_in_current_profile_(0.0_s),
      profile_loaded_(false) {
    LOG_DEBUG(MODULE_NAME, "TrajectoryInterpolator created.");
}

void TrajectoryInterpolator::loadSegment(const TrajectoryPoint& start_of_segment,
                                         const TrajectoryPoint& target_for_segment) {
    current_profile_.reset(); 
    time_in_current_profile_ = 0.0_s;
    profile_loaded_ = false;
    segment_target_metadata_provider_ = target_for_segment;
    MotionType segment_motion_type = target_for_segment.header.motion_type;
    LOG_INFO_F(MODULE_NAME, "Loading new segment. Type: %d, SpeedRatio: %.2f, AccelRatio: %.2f",
               static_cast<int>(segment_motion_type),
               target_for_segment.command.speed_ratio,
               target_for_segment.command.acceleration_ratio);

    double v_factor = std::max(0.01, std::min(1.0, target_for_segment.command.speed_ratio));
    double a_factor = std::max(0.01, std::min(1.0, target_for_segment.command.acceleration_ratio));

    try {
        if (segment_motion_type == MotionType::JOINT || segment_motion_type == MotionType::PTP) {
            RadiansPerSecond joint_v_max = DEFAULT_JOINT_V_MAX * v_factor;
            RadiansPerSecondSq joint_a_max = DEFAULT_JOINT_A_MAX * a_factor;
            LOG_DEBUG_F(MODULE_NAME, "JOINT/PTP profile: Vmax_eff = %s, Amax_eff = %s",
                        joint_v_max.toString().c_str(), joint_a_max.toString().c_str());
            current_profile_ = std::make_unique<TrapProfileJoint>(
                start_of_segment.command.joint_target, 
                target_for_segment.command.joint_target,
                joint_v_max, joint_a_max);
        } else if (segment_motion_type == MotionType::LIN) {
            MetersPerSecond cart_v_max = DEFAULT_CART_V_MAX * v_factor;
            MetersPerSecondSq cart_a_max = DEFAULT_CART_A_MAX * a_factor;
            LOG_DEBUG_F(MODULE_NAME, "LIN profile: Vmax_eff = %s, Amax_eff = %s",
                        cart_v_max.toString().c_str(), cart_a_max.toString().c_str());
            current_profile_ = std::make_unique<TrapProfileLIN>(
                start_of_segment.command.cartesian_target, 
                target_for_segment.command.cartesian_target,
                cart_v_max, cart_a_max);
        } else {
            LOG_ERROR_F(MODULE_NAME, "Unsupported motion type for segment: %d", static_cast<int>(segment_motion_type));
            throw std::invalid_argument("TrajectoryInterpolator: Unsupported motion type " + std::to_string(static_cast<int>(segment_motion_type)));
        }
    } catch (const std::exception& e) { /* ... (log and rethrow) ... */ 
        LOG_CRITICAL_F(MODULE_NAME, "Error creating motion profile: %s", e.what());
        current_profile_.reset(); profile_loaded_ = false; throw;
    }

    if (current_profile_) {
        profile_loaded_ = true;
        LOG_INFO_F(MODULE_NAME, "Segment loaded. Profile type: %d, Duration: %s",
                   static_cast<int>(current_profile_->type()), current_profile_->duration().toString().c_str());
        bool has_significant_move = false;
        if (current_profile_->type() == MotionType::JOINT) {
            auto* joint_profile = dynamic_cast<TrapProfileJoint*>(current_profile_.get());
            if (joint_profile && joint_profile->hasSignificantMovement()) has_significant_move = true;
        } else if (current_profile_->type() == MotionType::LIN) {
            auto* lin_profile = dynamic_cast<TrapProfileLIN*>(current_profile_.get());
            if (lin_profile && (lin_profile->hasTranslationalMovement() || lin_profile->hasOrientationChange())) has_significant_move = true;
        }
        if (current_profile_->duration() < (UnitConstants::DEFAULT_EPSILON * 1.0_s) && has_significant_move) { // Use Unit-based epsilon for Seconds
            LOG_WARN_F(MODULE_NAME, "Loaded segment type %d has near-zero duration but implies move. Check limits/points.", static_cast<int>(current_profile_->type()));
        }
        segment_final_point_cache_ = generateTrajectoryPointForTime(current_profile_->duration());
        segment_final_point_cache_.header.is_target_reached_for_this_point = true;
        segment_final_point_cache_.header.has_error_at_this_point = false;
        if (segment_motion_type == MotionType::JOINT || segment_motion_type == MotionType::PTP) {
            segment_final_point_cache_.command.joint_target = target_for_segment.command.joint_target;
            segment_final_point_cache_.command.cartesian_target = target_for_segment.command.cartesian_target;
        } else if (segment_motion_type == MotionType::LIN) {
            segment_final_point_cache_.command.cartesian_target = target_for_segment.command.cartesian_target;
            segment_final_point_cache_.command.joint_target = target_for_segment.command.joint_target;
        }
    } else { /* ... (log critical and throw logic_error) ... */ 
        LOG_CRITICAL(MODULE_NAME, "Profile creation resulted in null unique_ptr unexpectedly.");
        throw std::logic_error("TrajectoryInterpolator: MotionProfile unique_ptr is null after creation attempt.");
    }
}

TrajectoryPoint TrajectoryInterpolator::nextPoint(Seconds dt_step) {
    if (!profile_loaded_ || !current_profile_) {
        LOG_ERROR(MODULE_NAME, "nextPoint called but no profile is loaded.");
        throw std::logic_error("TrajectoryInterpolator::nextPoint: No profile loaded.");
    }
    if (dt_step <= 0.0_s) {
        LOG_DEBUG_F(MODULE_NAME, "nextPoint with dt_step <= 0. Returning current point at t = %s", time_in_current_profile_.toString().c_str());
        return generateTrajectoryPointForTime(time_in_current_profile_);
    }
    // Use Seconds::abs() for comparison
    if (time_in_current_profile_ >= current_profile_->duration() || 
        (current_profile_->duration() - time_in_current_profile_).abs() < (UnitConstants::DEFAULT_EPSILON * 1.0_s) ) {
        LOG_DEBUG_F(MODULE_NAME, "Profile already at end (t=%s, dur=%s). Returning cached final.",
                    time_in_current_profile_.toString().c_str(), current_profile_->duration().toString().c_str());
        return segment_final_point_cache_;
    }
    time_in_current_profile_ += dt_step;
    if (time_in_current_profile_ >= current_profile_->duration() ||
        (current_profile_->duration() - time_in_current_profile_).abs() < (UnitConstants::DEFAULT_EPSILON * 1.0_s) ) {
        time_in_current_profile_ = current_profile_->duration();
        LOG_DEBUG_F(MODULE_NAME, "Profile reached end at t=%s. Returning cached final.", time_in_current_profile_.toString().c_str());
        return segment_final_point_cache_;
    }
    
    return generateTrajectoryPointForTime(time_in_current_profile_);
}

bool TrajectoryInterpolator::isIdle() const {
    if (!profile_loaded_ || !current_profile_) return true;
    // Use Seconds::abs()
    return time_in_current_profile_ >= current_profile_->duration() ||
           (current_profile_->duration() - time_in_current_profile_).abs().value() < UnitConstants::DEFAULT_EPSILON;
}
Seconds TrajectoryInterpolator::getCurrentTimeInProfile() const { return time_in_current_profile_; }
Seconds TrajectoryInterpolator::getCurrentProfileDuration() const {
    if (profile_loaded_ && current_profile_) return current_profile_->duration();
    return 0.0_s;
}
TrajectoryPoint TrajectoryInterpolator::generateTrajectoryPointForTime(Seconds t) const { /* ... (same as previous refactored version) ... */ 
    if (!current_profile_) { 
        LOG_CRITICAL(MODULE_NAME, "generateTrajectoryPointForTime called with null profile!");
        throw std::logic_error("TrajectoryInterpolator::generateTrajectoryPointForTime: current_profile_ is null.");
    }
    TrajectoryPoint tp_out;
    tp_out.header = segment_target_metadata_provider_.header;
    tp_out.command.speed_ratio = segment_target_metadata_provider_.command.speed_ratio;
    tp_out.command.acceleration_ratio = segment_target_metadata_provider_.command.acceleration_ratio;
    tp_out.header.is_target_reached_for_this_point = false; 
    MotionType actual_profile_motion_type = current_profile_->type();
    if (actual_profile_motion_type == MotionType::JOINT) {
        tp_out.command.joint_target = current_profile_->interpolateJoints(t);
        tp_out.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
        LOG_DEBUG_F(MODULE_NAME,std::format("Interpolating point at t = {}; point={}", time_in_current_profile_.toString(), tp_out.command.joint_target.toJointPoseString()).c_str() );
    } else if (actual_profile_motion_type == MotionType::LIN) {
        tp_out.command.cartesian_target = current_profile_->interpolateCartesian(t);
        tp_out.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
        //LOG_DEBUG_F(MODULE_NAME, std::format("Interpolating point at t = {}; point={}", time_in_current_profile_.toString(), tp_out.command.cartesian_target.toDescriptiveString()));
    } else {
        LOG_CRITICAL_F(MODULE_NAME, "Unsupported profile type (%d) in generate.", static_cast<int>(actual_profile_motion_type));
        throw std::runtime_error("TrajectoryInterpolator: Unsupported profile type during point generation.");
    }
    
    return tp_out;
}

} // namespace RDT