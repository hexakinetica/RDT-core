// KinematicModel.h
#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "Logger.h"

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>
#include <string>
#include <vector>
#include <array>
#include <utility> // Для std::pair

namespace RDT {


class KinematicModel {
public:
    /**
     * @brief Creates and returns a KinematicModel for the KUKA KR6 R900 robot.
     */
    [[nodiscard]] static KinematicModel createKR6R900();

    // --- Существующие геттеры ---
    [[nodiscard]] const KDL::Chain& getChain() const { return chain_; }
    [[nodiscard]] const KDL::JntArray& getKdlJointMinLimits() const { return joint_min_limits_kdl_; }
    [[nodiscard]] const KDL::JntArray& getKdlJointMaxLimits() const { return joint_max_limits_kdl_; }
    [[nodiscard]] const AxisSet& getHomePositionJoints() const { return home_position_joints_; }

     /**
     * @brief Gets the structure containing all physical limits of the robot.
     * @return A const reference to the RobotLimits structure.
     */
    [[nodiscard]] const RobotLimits& getLimits() const { return limits_; }
    
    void setHomePositionJoints(const AxisSet& home_joints) { home_position_joints_ = home_joints; }

private:
    KinematicModel() = default;

    KDL::Chain chain_;
    KDL::JntArray joint_min_limits_kdl_;    // Хранит лимиты в радианах для KDL
    KDL::JntArray joint_max_limits_kdl_;    // Хранит лимиты в радианах для KDL
    AxisSet home_position_joints_;

     RobotLimits limits_; //Degrees and DegreesPerSecond

    static inline const std::string MODULE_NAME = "KinematicModel";
};

} // namespace RDT
#endif // KINEMATICMODEL_H