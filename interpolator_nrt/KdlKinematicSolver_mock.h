// KdlKinematicSolver_mock.h
#ifndef KDL_KINEMATIC_SOLVER_MOCK_H
#define KDL_KINEMATIC_SOLVER_MOCK_H

#include "DataTypes.h" // RDT::AxisSet, RDT::CartPose, RDT::AxisId
#include "Units.h"     // RDT::Radians, RDT::Meters, RDT::literals
#include "Logger.h"    // RDT::Logger
#include <optional>
#include <array>
#include <iostream> // For std::cout in debug (remove for production mock)

namespace RDT { // MODIFIED: Mock class now in RDT namespace

class KdlKinematicSolver_mock {
public:
    KdlKinematicSolver_mock() {
        LOG_DEBUG("KdlMock", "KdlKinematicSolver_mock created.");
    }

    std::optional<CartPose> solveFK(const AxisSet& joint_positions) {
        CartPose result_pose;
        // Use literals from RDT namespace (already active due to class being in RDT)
        // or qualify with RDT::literals:: if needed.
        // Since 'using namespace RDT::literals;' is in DataTypes.h and we include it,
        // _m and _rad should be available. If not, add `using namespace RDT::literals;` here.
        using namespace RDT::literals;


        // MODIFIED: Use operator[] which takes AxisId
        // And ensure correct multiplication: double * UnitType -> UnitType
        result_pose.x = joint_positions[AxisId::A1].angle.value() * 0.1_m; // double * Meters -> Meters
        result_pose.y = joint_positions[AxisId::A2].angle.value() * 0.1_m; // double * Meters -> Meters
        result_pose.z = 0.5_m;
        result_pose.rx = 0.0_rad;
        result_pose.ry = joint_positions[AxisId::A4].angle; // Assign Radians to Radians
        result_pose.rz = 0.0_rad;
        // LOG_DEBUG_F("KdlMock", "Mock FK for J1=%s -> X=%s",
        //             joint_positions[AxisId::A1].angle.toString().c_str(),
        //             result_pose.x.toString().c_str());
        return result_pose;
    }

    std::optional<AxisSet> solveIK(const CartPose& cartesian_pose, const AxisSet& seed_joint_positions) {
        AxisSet result_joints;
        using namespace RDT::literals;

        std::array<Radians, ROBOT_AXES_COUNT> angles;
        // cartesian_pose.x is RDT::Meters, .value() is double.
        // double * RDT::Radians -> RDT::Radians.
        angles[0] = cartesian_pose.x.value() * 1.0_rad;
        angles[1] = cartesian_pose.y.value() * 1.0_rad;
        for(size_t i = 2; i < ROBOT_AXES_COUNT; ++i) {
            // seed_joint_positions[AxisId].angle is RDT::Radians
            // 0.1_rad is RDT::Radians
            // (0.1_rad * double) is RDT::Radians
            // RDT::Radians + RDT::Radians is RDT::Radians
            angles[i] = seed_joint_positions[static_cast<AxisId>(i)].angle + (0.1_rad * static_cast<double>(i));
        }
        result_joints.fromAngleArray(angles);
        // LOG_DEBUG_F("KdlMock", "Mock IK for X=%s -> J1=%s",
        //             cartesian_pose.x.toString().c_str(),
        //             result_joints[AxisId::A1].angle.toString().c_str());
        return result_joints;
    }
};

} // namespace RDT
#endif // KDL_KINEMATIC_SOLVER_MOCK_H
