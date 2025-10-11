// KinematicModel.cpp
#include "KinematicModel.h"

namespace RDT {

// For convenience with literals if needed inside implementations
using namespace RDT::literals;

KinematicModel KinematicModel::createKR6R900() {
    LOG_INFO(MODULE_NAME, "Creating KinematicModel for KUKA KR6 R900.");
    KinematicModel model;
    KDL::Chain& chain = model.chain_; // Reference for convenience

    // --- Geometry Definition (DH parameters implicitly defined by KDL segments) ---
    // All segment link lengths are in meters, consistent with KDL's expectations.
    // KDL defines joints at the *base* of the segment they actuate.

    // Link 0 to Joint 1 (A1 axis of rotation)
    chain.addSegment(KDL::Segment("Link1_J0", KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(0.025, 0.0, 0.400))));
    // Link 1 to Joint 2 (A2 axis of rotation)
    chain.addSegment(KDL::Segment("Link2_J1", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, 0.455))));
    // Link 2 to Joint 3 (A3 axis of rotation)
    chain.addSegment(KDL::Segment("Link3_J2", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, -0.035))));
    // Link 3 to Joint 4 (A4 axis of rotation)
    chain.addSegment(KDL::Segment("Link4_J3", KDL::Joint(KDL::Joint::RotX),
                                  KDL::Frame(KDL::Vector(0.420, 0.0, 0.0))));
    // Link 4 to Joint 5 (A5 axis of rotation)
    chain.addSegment(KDL::Segment("Link5_J4", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.080, 0.0, 0.0))));
    // Link 5 to Joint 6 (A6 axis of rotation) - to Flange Center Point (FCP)
    chain.addSegment(KDL::Segment("Link6_Flange_J5", KDL::Joint(KDL::Joint::RotX),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, 0.0))));


    const std::size_t dof = chain.getNrOfJoints();
    if (dof != ROBOT_AXES_COUNT) {
        LOG_CRITICAL_F(MODULE_NAME, "KDL chain DOF (%zu) does not match ROBOT_AXES_COUNT (%zu).", dof, ROBOT_AXES_COUNT);
        throw std::logic_error("KinematicModel: DOF mismatch.");
    }

    model.joint_min_limits_kdl_.resize(dof);
    model.joint_max_limits_kdl_.resize(dof);

    // =================================================================================
    // *** NEW BLOCK: Define all physical limits in the RobotLimits structure ***
    // =================================================================================
    // Define joint position limits in degrees
    model.limits_.joint_position_limits_deg = {{
        { -170.0_deg, 170.0_deg },  // A1
        { -190.0_deg, 45.0_deg },   // A2
        { -120.0_deg, 156.0_deg },  // A3
        { -185.0_deg, 185.0_deg },  // A4
        { -120.0_deg, 120.0_deg },  // A5
        { -350.0_deg, 350.0_deg }   // A6
    }};

    // Define joint velocity limits in degrees per second (typical values for this robot class)
    model.limits_.joint_velocity_limits_deg_s = {
        150.0_deg_s, // A1
        150.0_deg_s, // A2
        150.0_deg_s, // A3
        300.0_deg_s, // A4 (wrist axes are typically faster)
        300.0_deg_s, // A5
        300.0_deg_s  // A6
    };
    
    // =================================================================================
    // *** MODIFIED BLOCK: Populate KDL-specific limits and home position from the new structure ***
    // =================================================================================
    std::array<Radians, ROBOT_AXES_COUNT> home_angles_rad;

    for (std::size_t i = 0; i < dof; ++i) {
        // Take limits from our new structure and convert them to radians for KDL
        const auto& pos_limits = model.limits_.joint_position_limits_deg[i];
        model.joint_min_limits_kdl_(i) = pos_limits.first.toRadians().value();
        model.joint_max_limits_kdl_(i) = pos_limits.second.toRadians().value();
        
        // Define home position (all joints at 0.0)
        home_angles_rad[i] = 0.0_rad;
    }
    model.home_position_joints_.fromAngleArray(home_angles_rad);

    LOG_INFO_F(MODULE_NAME, "KUKA KR6 R900 model created with %zu DoF.", dof);
    LOG_DEBUG_F(MODULE_NAME, "Home position (deg): %s", model.home_position_joints_.toJointPoseString().c_str());

    return model;
}

} // namespace RDT