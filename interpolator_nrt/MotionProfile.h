// MotionProfile.h
#ifndef MOTIONPROFILE_H
#define MOTIONPROFILE_H

#pragma once

#include "DataTypes.h" // RDT::AxisSet, RDT::CartPose, RDT::MotionType
#include "Units.h"     // RDT::Seconds, RDT::Radians, RDT::Meters, etc.
#include "Logger.h"    // RDT::Logger (  .h     ,    inline )
#include <Eigen/Geometry> // For Eigen::Quaterniond in LIN profile
#include <memory>    // For std::unique_ptr in TrajectoryInterpolator
#include <string>    // For std::string
#include <stdexcept> // For std::runtime_error, std::invalid_argument

namespace RDT {

//      
using namespace RDT::literals;

//       -   TrajectoryInterpolator.h

class MotionProfile {
public:
    virtual ~MotionProfile() = default;
    [[nodiscard]] virtual Seconds duration() const = 0;
    [[nodiscard]] virtual MotionType type() const = 0;
    [[nodiscard]] virtual double programmedSpeedValue() const = 0;
    [[nodiscard]] virtual bool isDone(Seconds t) const {
        return t >= duration() || (duration() - t).abs() < (UnitConstants::DEFAULT_EPSILON * 1.0_s);
    }
    [[nodiscard]] virtual AxisSet interpolateJoints(Seconds t) const;
    [[nodiscard]] virtual CartPose interpolateCartesian(Seconds t) const;
};

struct TrapParams {
    Seconds t_accel = 0.0_s;
    Seconds t_const_vel = 0.0_s;
    Seconds t_decel = 0.0_s;
    double peak_vel_value = 0.0;
    Seconds total_duration = 0.0_s;
    double actual_accel_value = 0.0;
};

class TrapezoidalProfileMath {
public:
    TrapezoidalProfileMath();
    void calculateTrapezoid(double displacement_val, double v_limit_val, double a_limit_val);
    [[nodiscard]] double getPathPositionAtTime(Seconds t) const;
    [[nodiscard]] double getCurrentSpeedAtTime(Seconds t) const;
    [[nodiscard]] const TrapParams& getParams() const;
    [[nodiscard]] double getTotalDisplacement() const;
private:
    TrapParams params_;
    double displacement_val_ = 0.0;
    double v_limit_val_ = 0.0;
    double a_limit_val_ = 0.0;
};

class TrapProfileJoint : public MotionProfile {
public:
    TrapProfileJoint(const AxisSet& start_config, const AxisSet& end_config,
                     RadiansPerSecond v_max_leading_axis, RadiansPerSecondSq a_max_leading_axis);
    [[nodiscard]] AxisSet interpolateJoints(Seconds t) const override;
    [[nodiscard]] Seconds duration() const override;
    [[nodiscard]] MotionType type() const override;
    [[nodiscard]] double programmedSpeedValue() const override;
    // MODIFIED: Made public
    [[nodiscard]] bool hasSignificantMovement() const;
private:
    AxisSet start_joints_, end_joints_;
    RadiansPerSecond v_max_limit_leading_;
    RadiansPerSecondSq a_max_limit_leading_;
    Radians max_angular_delta_ = 0.0_rad;
    TrapezoidalProfileMath profile_math_;
};

class TrapProfileLIN : public MotionProfile {
public:
    TrapProfileLIN(const CartPose& start_cart_pose, const CartPose& end_cart_pose,
                   MetersPerSecond v_max_cartesian, MetersPerSecondSq a_max_cartesian);
    [[nodiscard]] CartPose interpolateCartesian(Seconds t) const override;
    [[nodiscard]] Seconds duration() const override;
    [[nodiscard]] MotionType type() const override;
    [[nodiscard]] double programmedSpeedValue() const override;
    // MODIFIED: Made public
    [[nodiscard]] bool hasTranslationalMovement() const;
    [[nodiscard]] bool hasOrientationChange() const;
private:
    [[nodiscard]] static Eigen::Quaterniond poseToQuaternion(const CartPose& pose);
    static void quaternionToPoseRot(const Eigen::Quaterniond& q, CartPose& pose);
    CartPose start_pose_cart_, end_pose_cart_;
    MetersPerSecond v_max_limit_;
    MetersPerSecondSq a_max_limit_;
    Meters s_total_displacement_ = 0.0_m;
    Eigen::Quaterniond q_start_, q_end_slerp_target_;
    TrapezoidalProfileMath profile_math_;
    bool only_orientation_change_ = false;
};

} // namespace RDT
#endif // MOTIONPROFILE_H