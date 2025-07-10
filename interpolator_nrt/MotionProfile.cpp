// MotionProfile.cpp
#include "MotionProfile.h"
#include "Logger.h" // For RDT::Logger
#include <algorithm> // For std::max, std::min
#include <cmath>     // For std::abs, std::sqrt, std::fmod

namespace RDT {

// For convenience with literals if needed inside implementations
using namespace RDT::literals;

// --- MotionProfile Base ---
AxisSet MotionProfile::interpolateJoints(Seconds t) const {
    (void)t; 
    LOG_ERROR_F("MotionProfile", "interpolateJoints called on a profile of type %d that does not support it.", static_cast<int>(type()));
    throw std::logic_error("This motion profile does not support joint interpolation.");
}

CartPose MotionProfile::interpolateCartesian(Seconds t) const {
    (void)t; 
    LOG_ERROR_F("MotionProfile", "interpolateCartesian called on a profile of type %d that does not support it.", static_cast<int>(type()));
    throw std::logic_error("This motion profile does not support Cartesian interpolation.");
}

// --- TrapezoidalProfileMath ---
TrapezoidalProfileMath::TrapezoidalProfileMath() { /* params_ default constructed */ }

void TrapezoidalProfileMath::calculateTrapezoid(double displacement_val, double v_limit_val, double a_limit_val) {
    params_ = {}; 
    displacement_val_ = std::abs(displacement_val);
    v_limit_val_ = std::abs(v_limit_val);
    a_limit_val_ = std::abs(a_limit_val);

    if (displacement_val_ < UnitConstants::DEFAULT_EPSILON) {
        LOG_DEBUG("TrapMath", "Zero displacement, profile duration is zero.");
        return; 
    }

    if (v_limit_val_ < UnitConstants::DEFAULT_EPSILON) {
        LOG_WARN("TrapMath", "Velocity limit is zero but displacement is non-zero. Profile duration is zero (impossible move).");
        return;
    }
    if (a_limit_val_ < UnitConstants::DEFAULT_EPSILON) {
        LOG_DEBUG("TrapMath", "Acceleration limit is zero. Calculating as constant velocity profile.");
        params_.t_accel = 0.0_s;
        params_.t_decel = 0.0_s;
        params_.t_const_vel = Seconds(displacement_val_ / v_limit_val_);
        params_.peak_vel_value = v_limit_val_;
        params_.actual_accel_value = 0.0;
        params_.total_duration = params_.t_const_vel;
        return;
    }

    Seconds t_acc_to_v_limit = Seconds(v_limit_val_ / a_limit_val_);
    double s_acc_dec_full_v_limit = v_limit_val_ * v_limit_val_ / a_limit_val_;

    if (s_acc_dec_full_v_limit <= displacement_val_ + UnitConstants::DEFAULT_EPSILON) {
        params_.peak_vel_value = v_limit_val_;
        params_.t_accel = t_acc_to_v_limit;
        params_.t_decel = t_acc_to_v_limit;
        params_.t_const_vel = Seconds((displacement_val_ - s_acc_dec_full_v_limit) / v_limit_val_);
        params_.actual_accel_value = a_limit_val_;
    } else { 
        params_.t_accel = Seconds(std::sqrt(displacement_val_ / a_limit_val_));
        params_.t_decel = params_.t_accel;
        params_.peak_vel_value = a_limit_val_ * params_.t_accel.value();
        params_.t_const_vel = 0.0_s;
        params_.actual_accel_value = a_limit_val_;
    }

    if (params_.t_const_vel < 0.0_s) params_.t_const_vel = 0.0_s;
    params_.total_duration = params_.t_accel + params_.t_const_vel + params_.t_decel;

    if (params_.total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s) && displacement_val_ > UnitConstants::DEFAULT_EPSILON) {
        LOG_WARN_F("TrapMath", "Calculated near-zero duration (%.3f ms) for non-zero displacement (%.4f). Vlim: %.4f, Alim: %.4f.",
            params_.total_duration.value()*1000.0, displacement_val_, v_limit_val_, a_limit_val_);
    }
}

double TrapezoidalProfileMath::getPathPositionAtTime(Seconds t) const { /* ... (same as previous full version, uses .value() for Seconds) ... */ 
    if (displacement_val_ < UnitConstants::DEFAULT_EPSILON || params_.total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
        return (t >= params_.total_duration) ? displacement_val_ : 0.0;
    }
    Seconds clamped_t = std::max(0.0_s, std::min(t, params_.total_duration));
    double time_s = clamped_t.value();
    if (clamped_t <= params_.t_accel) {
        return 0.5 * params_.actual_accel_value * time_s * time_s;
    } else if (clamped_t <= params_.t_accel + params_.t_const_vel) {
        double s_at_accel_end = 0.5 * params_.actual_accel_value * params_.t_accel.value() * params_.t_accel.value();
        return s_at_accel_end + params_.peak_vel_value * (time_s - params_.t_accel.value());
    } else {
        double s_at_accel_end = 0.5 * params_.actual_accel_value * params_.t_accel.value() * params_.t_accel.value();
        double s_at_const_end = s_at_accel_end + params_.peak_vel_value * params_.t_const_vel.value();
        double time_in_decel = time_s - (params_.t_accel.value() + params_.t_const_vel.value());
        return s_at_const_end + (params_.peak_vel_value * time_in_decel - 0.5 * params_.actual_accel_value * time_in_decel * time_in_decel);
    }
}
double TrapezoidalProfileMath::getCurrentSpeedAtTime(Seconds t) const { /* ... (same as previous full version, uses .value() for Seconds) ... */
    if (displacement_val_ < UnitConstants::DEFAULT_EPSILON || params_.total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s)) return 0.0;
    Seconds clamped_t = std::max(0.0_s, std::min(t, params_.total_duration));
    double time_s = clamped_t.value();
    if (clamped_t <= params_.t_accel) {
        return params_.actual_accel_value * time_s;
    } else if (clamped_t <= params_.t_accel + params_.t_const_vel) {
        return params_.peak_vel_value;
    } else {
        double time_in_decel = time_s - (params_.t_accel.value() + params_.t_const_vel.value());
        return std::max(0.0, params_.peak_vel_value - params_.actual_accel_value * time_in_decel);
    }
}
const TrapParams& TrapezoidalProfileMath::getParams() const { return params_; }
double TrapezoidalProfileMath::getTotalDisplacement() const { return displacement_val_; }

// --- TrapProfileJoint ---
TrapProfileJoint::TrapProfileJoint(const AxisSet& start_config, const AxisSet& end_config,
                                   RadiansPerSecond v_max_leading_axis, RadiansPerSecondSq a_max_leading_axis)
    : start_joints_(start_config), end_joints_(end_config),
      v_max_limit_leading_(v_max_leading_axis), a_max_limit_leading_(a_max_leading_axis) {
    if (v_max_limit_leading_ <= 0.0_rad_s && hasSignificantMovement()) {
        LOG_ERROR("TrapProfileJoint", "v_max_leading_axis must be positive if there is movement.");
        throw std::invalid_argument("TrapProfileJoint: v_max_leading_axis must be positive for movement.");
    }
    if (a_max_limit_leading_ <= 0.0_rad_s2 && hasSignificantMovement() && v_max_limit_leading_ > 0.0_rad_s) {
        LOG_ERROR("TrapProfileJoint", "a_max_leading_axis must be positive if there is movement and v_max > 0.");
        throw std::invalid_argument("TrapProfileJoint: a_max_leading_axis must be positive for movement with v_max > 0.");
    }
    Radians max_delta_val = 0.0_rad;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        Radians delta = (end_joints_.at(i).angle - start_joints_.at(i).angle).abs(); // Use Unit.abs()
        if (delta > max_delta_val) max_delta_val = delta;
    }
    max_angular_delta_ = max_delta_val;
    if (max_angular_delta_ > (UnitConstants::DEFAULT_EPSILON * 1.0_rad) ) { // Compare with Radian epsilon
        profile_math_.calculateTrapezoid(max_angular_delta_.value(), v_max_limit_leading_.value(), a_max_limit_leading_.value());
        if (profile_math_.getParams().total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
             LOG_WARN_F("TrapProfileJoint", "Calculated near-zero duration for significant joint movement (max_delta: %s). Check limits.", max_angular_delta_.toString().c_str());
        }
    } else { LOG_DEBUG("TrapProfileJoint", "No significant joint movement. Duration is zero."); }
}
AxisSet TrapProfileJoint::interpolateJoints(Seconds t) const { /* ... (same as previous full version, uses .value() for math, assign to Unit types) ... */
    double s_path_norm_factor = 0.0;
    if (max_angular_delta_ > (UnitConstants::DEFAULT_EPSILON * 1.0_rad) &&
        profile_math_.getParams().total_duration > (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
        s_path_norm_factor = profile_math_.getPathPositionAtTime(t) / max_angular_delta_.value();
    } else { s_path_norm_factor = (t >= duration()) ? 1.0 : 0.0; }
    s_path_norm_factor = std::max(0.0, std::min(s_path_norm_factor, 1.0));
    AxisSet interp_joints;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        Radians start_angle = start_joints_.at(i).angle;
        Radians end_angle = end_joints_.at(i).angle;
        interp_joints.at(i).angle = start_angle + (end_angle - start_angle) * s_path_norm_factor;
        if (max_angular_delta_ > (UnitConstants::DEFAULT_EPSILON * 1.0_rad) &&
            profile_math_.getParams().total_duration > (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
            double leading_axis_speed_val = profile_math_.getCurrentSpeedAtTime(t);
            Radians joint_total_delta = end_angle - start_angle;
            double movement_ratio = (max_angular_delta_.value() < UnitConstants::DEFAULT_EPSILON) ? 0.0 : (joint_total_delta.value() / max_angular_delta_.value());
            interp_joints.at(i).velocity = RadiansPerSecond(leading_axis_speed_val * movement_ratio);
        } else { interp_joints.at(i).velocity = 0.0_rad_s; }
        interp_joints.at(i).acceleration = 0.0_rad_s2;
    }
    return interp_joints;
}
Seconds TrapProfileJoint::duration() const { return profile_math_.getParams().total_duration; }
MotionType TrapProfileJoint::type() const { return MotionType::JOINT; }
double TrapProfileJoint::programmedSpeedValue() const { return v_max_limit_leading_.value(); }
bool TrapProfileJoint::hasSignificantMovement() const { /* ... (same as previous full version, using Unit.abs() > UnitEpsilon ) ... */
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        if ((end_joints_.at(i).angle - start_joints_.at(i).angle).abs() > (UnitConstants::DEFAULT_EPSILON * 1.0_rad)) {
            return true;
        }
    }
    return false;
}

// --- TrapProfileLIN ---
TrapProfileLIN::TrapProfileLIN(const CartPose& scp, const CartPose& ecp, MetersPerSecond v_max_c, MetersPerSecondSq a_max_c)
    : start_pose_cart_(scp), end_pose_cart_(ecp), v_max_limit_(v_max_c), a_max_limit_(a_max_c) {
    if (v_max_limit_ <= 0.0_m_s && (hasTranslationalMovement() || hasOrientationChange()) ) { /* LOG_ERROR, throw */ }
    if (a_max_limit_ <= 0.0_m_s2 && (hasTranslationalMovement() || hasOrientationChange()) && v_max_limit_ > 0.0_m_s) { /* LOG_ERROR, throw */ }
    Meters dx = end_pose_cart_.x - start_pose_cart_.x; Meters dy = end_pose_cart_.y - start_pose_cart_.y; Meters dz = end_pose_cart_.z - start_pose_cart_.z;
    s_total_displacement_ = Meters(std::sqrt(dx.value()*dx.value() + dy.value()*dy.value() + dz.value()*dz.value()));
    q_start_ = poseToQuaternion(start_pose_cart_); Eigen::Quaterniond q_end_temp = poseToQuaternion(end_pose_cart_);
    if (q_start_.dot(q_end_temp) < 0.0) q_end_slerp_target_.coeffs() = -q_end_temp.coeffs(); else q_end_slerp_target_ = q_end_temp;
    only_orientation_change_ = !hasTranslationalMovement() && hasOrientationChange();
    if (s_total_displacement_ > (UnitConstants::DEFAULT_EPSILON * 1.0_m)) {
        profile_math_.calculateTrapezoid(s_total_displacement_.value(), v_max_limit_.value(), a_max_limit_.value());
    } else if (only_orientation_change_) {
        profile_math_.calculateTrapezoid(1.0, v_max_limit_.value(), a_max_limit_.value());
        LOG_DEBUG("TrapProfileLIN", "Pure orientation change. Profile calculated based on dummy displacement.");
    } else { LOG_DEBUG("TrapProfileLIN", "No significant movement. Duration is zero."); }
    if (profile_math_.getParams().total_duration < (UnitConstants::DEFAULT_EPSILON * 1.0_s) && (hasTranslationalMovement() || hasOrientationChange())) {
        LOG_WARN("TrapProfileLIN", "Near-zero duration for LIN segment with movement/orientation change.");
    }
}
CartPose TrapProfileLIN::interpolateCartesian(Seconds t) const { /* ... (same as previous full version, uses .value() for math, assign to Unit types) ... */
    double alpha = 0.0;
    if (profile_math_.getParams().total_duration > (UnitConstants::DEFAULT_EPSILON * 1.0_s)) {
        if (s_total_displacement_ > (UnitConstants::DEFAULT_EPSILON * 1.0_m)) {
            double s_path_on_profile = profile_math_.getPathPositionAtTime(t);
            alpha = (s_total_displacement_.value() < UnitConstants::DEFAULT_EPSILON) ? 1.0 : (s_path_on_profile / s_total_displacement_.value());
        } else if (only_orientation_change_) {
            alpha = std::min(t, profile_math_.getParams().total_duration).value() / profile_math_.getParams().total_duration.value();
        } else { alpha = (t >= duration()) ? 1.0 : 0.0; }
    } else { alpha = (t >= duration()) ? 1.0 : 0.0; }
    alpha = std::max(0.0, std::min(alpha, 1.0));
    CartPose p_interp;
    p_interp.x = start_pose_cart_.x + (end_pose_cart_.x - start_pose_cart_.x) * alpha;
    p_interp.y = start_pose_cart_.y + (end_pose_cart_.y - start_pose_cart_.y) * alpha;
    p_interp.z = start_pose_cart_.z + (end_pose_cart_.z - start_pose_cart_.z) * alpha;
    Eigen::Quaterniond q_interpolated = q_start_.slerp(alpha, q_end_slerp_target_);
    quaternionToPoseRot(q_interpolated.normalized(), p_interp);
    return p_interp;
}
Seconds TrapProfileLIN::duration() const { return profile_math_.getParams().total_duration; }
MotionType TrapProfileLIN::type() const { return MotionType::LIN; }
double TrapProfileLIN::programmedSpeedValue() const { return v_max_limit_.value(); }
bool TrapProfileLIN::hasTranslationalMovement() const { /* ... (same as previous, using Unit > UnitEpsilon ) ... */
    return s_total_displacement_ > (UnitConstants::DEFAULT_EPSILON * 1.0_m);
}
bool TrapProfileLIN::hasOrientationChange() const { /* ... (same as previous) ... */
    constexpr double QUAT_DOT_PRODUCT_FOR_IDENTITY_LOWER = 1.0 - UnitConstants::DEFAULT_EPSILON * 100;
    double dot_product = q_start_.dot(q_end_slerp_target_);
    return !(dot_product >= QUAT_DOT_PRODUCT_FOR_IDENTITY_LOWER);
}
Eigen::Quaterniond TrapProfileLIN::poseToQuaternion(const CartPose& pose) { /* ... same ... */ return Eigen::AngleAxisd(pose.rz.value(), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(pose.ry.value(), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(pose.rx.value(), Eigen::Vector3d::UnitX()); }
void TrapProfileLIN::quaternionToPoseRot(const Eigen::Quaterniond& q, CartPose& pose) { /* ... same ... */
    Eigen::Vector3d euler_angles_rad = q.toRotationMatrix().eulerAngles(2, 1, 0);
    pose.rz = Radians(euler_angles_rad[0]); pose.ry = Radians(euler_angles_rad[1]); pose.rx = Radians(euler_angles_rad[2]);
}

} // namespace RDT