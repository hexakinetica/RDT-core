// KinematicModel.h
#ifndef KINEMATICMODEL_H
#define KINEMATICMODEL_H

#pragma once

#include "DataTypes.h" // RDT::AxisSet and thus RDT::Units
#include "Units.h"     // RDT::Radians, RDT::Meters, RDT::UnitConstants for PI
#include "Logger.h"    // RDT::Logger

#include <kdl/chain.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp> // For KDL::Frame, KDL::Vector
#include <string>
#include <vector>      // For std::vector if used for limits internally before KDL::JntArray
#include <array>       // For std::array for joint_limits_deg

namespace RDT {

/**
 * @class KinematicModel
 * @brief Represents the kinematic structure and properties of a robot.
 *
 * This class encapsulates the KDL chain, joint limits, and a home position.
 * It provides factory methods for creating models of specific robots (e.g., KUKA KR6 R900).
 */
class KinematicModel {
public:
    /**
     * @brief Creates and returns a KinematicModel for the KUKA KR6 R900 robot.
     * Defines the DH parameters (implicitly via KDL segments), joint limits, and a default home position.
     * @return KinematicModel An instance configured for KUKA KR6 R900.
     */
    [[nodiscard]] static KinematicModel createKR6R900();

    // Public getters for model properties
    [[nodiscard]] const KDL::Chain& getChain() const { return chain_; }
    [[nodiscard]] const KDL::JntArray& getKdlJointMinLimits() const { return joint_min_limits_kdl_; }
    [[nodiscard]] const KDL::JntArray& getKdlJointMaxLimits() const { return joint_max_limits_kdl_; }
    [[nodiscard]] const AxisSet& getHomePositionJoints() const { return home_position_joints_; }

    // Allow setting a custom home position after creation if needed
    void setHomePositionJoints(const AxisSet& home_joints) { home_position_joints_ = home_joints; }


private:
    // Private constructor to enforce creation via factory methods like createKR6R900()
    KinematicModel() = default;

    KDL::Chain chain_;                     ///< KDL kinematic chain definition.
    KDL::JntArray joint_min_limits_kdl_;    ///< KDL array of minimum joint limits (radians).
    KDL::JntArray joint_max_limits_kdl_;    ///< KDL array of maximum joint limits (radians).
    AxisSet home_position_joints_;          ///< Robot's home position in RDT::AxisSet format (uses RDT::Radians).

    static inline const std::string MODULE_NAME = "KinematicModel"; // For logging
};

} // namespace RDT
#endif // KINEMATICMODEL_H