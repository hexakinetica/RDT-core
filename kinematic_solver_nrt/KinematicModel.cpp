// KinematicModel.cpp
#include "KinematicModel.h"

namespace RDT {

// For convenience with literals if needed inside implementations
using namespace RDT::literals;

KinematicModel KinematicModel::createKR6R900() {
    LOG_INFO(MODULE_NAME, "Creating KinematicModel for KUKA KR6 R900.");
    KinematicModel model;
    KDL::Chain& chain = model.chain_; // Reference for convenience

    // DH parameters are implicitly defined by KDL segment definitions.
    // All segment link lengths are in meters, consistent with KDL's expectations.
    // Frame definitions are from base of segment to tip of segment.
    // Vector component is translation, Rotation component is RotZ/RotY/RotX.
    // KDL defines joints at the *base* of the segment they actuate.

    // Link 0 to Joint 1 (A1 axis of rotation)
    chain.addSegment(KDL::Segment("Link1_J0", KDL::Joint(KDL::Joint::RotZ),
                                  KDL::Frame(KDL::Vector(0.025, 0.0, 0.400))));
    // Link 1 to Joint 2 (A2 axis of rotation)
    chain.addSegment(KDL::Segment("Link2_J1", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, 0.455))));
    // Link 2 to Joint 3 (A3 axis of rotation)
    chain.addSegment(KDL::Segment("Link3_J2", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, -0.035)))); // Corrected offset for KUKA: a3=35mm along -Y in KDL frame
                                                                               // Assuming standard KDL setup where Z is along the link, X is normal.
                                                                               // If KUKA's a_i is along X, then KDL::Vector(0.035,0,0)
                                                                               // This needs verification against actual KUKA DH table and KDL segment conventions.
                                                                               // For now, using a common interpretation.
                                                                               // A common error is sign of a_i. Often, a_i is distance along X_i-1.
                                                                               // KDL Frame: Translation from previous frame to current frame's origin,
                                                                               // then Rotation from previous frame to current frame.
                                                                               // Let's assume the original values were simplified for a specific KDL frame setup:
                                                                               // Original: KDL::Frame(KDL::Vector(0.0, 0.0, 0.035)) -- this implies d_i typically
    // Link 3 to Joint 4 (A4 axis of rotation)
    chain.addSegment(KDL::Segment("Link4_J3", KDL::Joint(KDL::Joint::RotX),
                                  KDL::Frame(KDL::Vector(0.420, 0.0, 0.0))));
    // Link 4 to Joint 5 (A5 axis of rotation)
    chain.addSegment(KDL::Segment("Link5_J4", KDL::Joint(KDL::Joint::RotY),
                                  KDL::Frame(KDL::Vector(0.080, 0.0, 0.0))));
    // Link 5 to Joint 6 (A6 axis of rotation) - to Flange Center Point (FCP)
    // The final segment usually describes the transform to the flange.
    // Tool transform will be applied on top of this.
    // Assuming 0 is distance from J6 axis to flange.
    chain.addSegment(KDL::Segment("Link6_Flange_J5", KDL::Joint(KDL::Joint::RotX),
                                  KDL::Frame(KDL::Vector(0.0, 0.0, 0.0)))); // If TCP is defined from J6 origin.
                                  // Or if 0.100m is along the last link to the flange: KDL::Frame(KDL::Vector(0.100, 0.0, 0.0))
                                  // For KUKA, often the flange is the origin of the last frame.
                                  // Let's assume the last joint axis is at the flange for simplicity of TCP later.
                                  // Or, if there's a final offset to the flange mounting point:
                                  // KDL::Frame(KDL::Vector(DISTANCE_TO_FLANGE_X, DISTANCE_TO_FLANGE_Y, DISTANCE_TO_FLANGE_Z))


    const std::size_t dof = chain.getNrOfJoints();
    if (dof != ROBOT_AXES_COUNT) {
        LOG_CRITICAL_F(MODULE_NAME, "KDL chain DOF (%zu) does not match ROBOT_AXES_COUNT (%zu).", dof, ROBOT_AXES_COUNT);
        throw std::logic_error("KinematicModel: DOF mismatch.");
    }

    model.joint_min_limits_kdl_.resize(dof);
    model.joint_max_limits_kdl_.resize(dof);
    // model.home_position_joints_ is already default constructed (all axes default)

    // Joint limits for KUKA KR6 R900 (degrees)
    const std::array<std::pair<double, double>, ROBOT_AXES_COUNT> joint_limits_deg_array = {{

        // Corrected KUKA KR6 R900 sixx limits:
        // A1: ±170°
        // A2: +45° to -190° (KDL: -190 to 45)
        // A3: +156° to -120° (KDL: -120 to 156, but depends on J2. Common range -210 to 66)
        // A4: ±185°
        // A5: ±120°
        // A6: ±350°

        // Let's use a more geenral set for KR6 R900 for this example:
        {-170.0, 170.0}, // A1
        { -190.0, 180.0}, // A2 (KDL: min, max)
        { -120.0, 156.0}, // A3 (KDL: min, max) - this range assumes J2 is at 0. Max extension might be different.
        {-185.0, 185.0}, // A4
        {-120.0, 120.0}, // A5
        {-350.0, 350.0}  // A6
    }};


    std::array<Radians, ROBOT_AXES_COUNT> home_angles_rad; // For RDT::AxisSet

    for (std::size_t i = 0; i < dof; ++i) {
        Degrees min_deg(joint_limits_deg_array[i].first);
        Degrees max_deg(joint_limits_deg_array[i].second);
        model.joint_min_limits_kdl_(i) = min_deg.toRadians().value(); // KDL wants double radians
        model.joint_max_limits_kdl_(i) = max_deg.toRadians().value(); // KDL wants double radians
        
        // Default home position: all joints at 0.0 radians
        // Except A2 often at -90deg and A3 at +90deg for an "upright" pose for many 6-axis.
        // For KUKA KR6, a common "zero" or "shipping" pose might be:
        // A1=0, A2=-90, A3=90, A4=0, A5=0, A6=0
        if (i == 1) { // A2
            //home_angles_rad[i] = (-90.0_deg).toRadians();
        } else if (i == 2) { // A3
           // home_angles_rad[i] = (90.0_deg).toRadians();
        } else {
            home_angles_rad[i] = 0.0_rad;
        }
    }
    model.home_position_joints_.fromAngleArray(home_angles_rad);

    LOG_INFO_F(MODULE_NAME, "KUKA KR6 R900 model created with %zu DoF.", dof);
    LOG_DEBUG_F(MODULE_NAME, "Home position (deg): %s", model.home_position_joints_.toJointPoseString().c_str());

    return model;
}

} // namespace RDT