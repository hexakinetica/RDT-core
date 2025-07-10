// KdlKinematicSolver.cpp
#include "KdlKinematicSolver.h"

namespace RDT {

// For convenience with literals if needed inside implementations
using namespace RDT::literals;

KdlKinematicSolver::KdlKinematicSolver(const KinematicModel& model)
    : kdl_chain_ref_(model.getChain()), // Store reference to chain from model
      kdl_joint_min_copy_(model.getKdlJointMinLimits()), // Copy KDL JntArrays
      kdl_joint_max_copy_(model.getKdlJointMaxLimits()),
      rdt_home_position_joints_(model.getHomePositionJoints()) { // Copy RDT AxisSet

    LOG_INFO_F(MODULE_NAME, "Initializing KDL Kinematic Solver for a chain with %u joints.", kdl_chain_ref_.getNrOfJoints());

    if (kdl_chain_ref_.getNrOfJoints() == 0) {
        LOG_CRITICAL(MODULE_NAME, "KDL chain has zero joints. Cannot initialize solver.");
        throw std::runtime_error("KdlKinematicSolver: KDL chain is empty.");
    }
    if (kdl_joint_min_copy_.rows() != kdl_chain_ref_.getNrOfJoints() ||
        kdl_joint_max_copy_.rows() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_CRITICAL(MODULE_NAME, "Joint limit array sizes do not match chain DOF.");
        throw std::runtime_error("KdlKinematicSolver: Joint limit size mismatch.");
    }


    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_ref_);
    if (!fk_solver_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to create KDL FK solver.");
        throw std::runtime_error("KdlKinematicSolver: FK solver creation failed.");
    }

    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_pinv>(kdl_chain_ref_);
    if (!ik_vel_solver_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to create KDL IK velocity solver.");
        throw std::runtime_error("KdlKinematicSolver: IK velocity solver creation failed.");
    }

    // KDL::ChainIkSolverPos_NR_JL(const Chain& chain, const JntArray& q_min, const JntArray& q_max, ...)
    ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        kdl_chain_ref_, kdl_joint_min_copy_, kdl_joint_max_copy_,
        *fk_solver_, *ik_vel_solver_,
        300,   // Max iterations
        1e-5); // Epsilon (tolerance for convergence) - was 1e-3, tighter might be better
    if (!ik_pos_solver_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to create KDL IK position solver.");
        throw std::runtime_error("KdlKinematicSolver: IK position solver creation failed.");
    }
    LOG_INFO(MODULE_NAME, "KDL Kinematic Solvers initialized successfully.");
}

CartPose KdlKinematicSolver::solveFK(const AxisSet& joints) const {
    if (joints.size() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_ERROR_F(MODULE_NAME, "FK input AxisSet size (%zu) does not match chain DOF (%u).", joints.size(), kdl_chain_ref_.getNrOfJoints());
        throw std::invalid_argument("KdlKinematicSolver::solveFK: Joint count mismatch.");
    }
    KDL::JntArray kdl_jnt_array = toKdlJntArray(joints);
    KDL::Frame kdl_frame_result;

    int fk_status = fk_solver_->JntToCart(kdl_jnt_array, kdl_frame_result);
    if (fk_status < 0) {
        LOG_ERROR_F(MODULE_NAME, "KDL FK computation failed with error code: %d", fk_status);
        throw std::runtime_error("KdlKinematicSolver::solveFK: KDL JntToCart failed.");
    }

    CartPose result_pose;
    // Position (KDL Vector is in meters)
    result_pose.x = Meters(kdl_frame_result.p.x());
    result_pose.y = Meters(kdl_frame_result.p.y());
    result_pose.z = Meters(kdl_frame_result.p.z());

    // Orientation (KDL GetRPY returns radians)
    double kdl_r, kdl_p, kdl_y;
    kdl_frame_result.M.GetRPY(kdl_r, kdl_p, kdl_y); // Roll, Pitch, Yaw
    result_pose.rx = Radians(kdl_r);
    result_pose.ry = Radians(kdl_p);
    result_pose.rz = Radians(kdl_y);

    LOG_DEBUG_F(MODULE_NAME, "FK solved. Joint: %s -> Cart: %s",
                joints.toJointPoseString().c_str(), result_pose.toDescriptiveString().c_str());
    return result_pose;
}

std::optional<AxisSet> KdlKinematicSolver::solveIK(const CartPose& pose,
                                                   const AxisSet& seed_joints,
                                                   const IKHint& hint) const {
    (void)hint; // IKHint is not used by KDL::ChainIkSolverPos_NR_JL directly. Log if needed.
    // LOG_DEBUG_F(MODULE_NAME, "IK Hint custom_solver_flags: %s", hint.custom_solver_flags.c_str());

    if (seed_joints.size() != kdl_chain_ref_.getNrOfJoints()) {
         LOG_ERROR_F(MODULE_NAME, "IK input seed_joints size (%zu) does not match chain DOF (%u).", seed_joints.size(), kdl_chain_ref_.getNrOfJoints());
        throw std::invalid_argument("KdlKinematicSolver::solveIK: Seed joint count mismatch.");
    }

    // Convert RDT::CartPose to KDL::Frame
    // KDL RPY expects radians. Our RDT::Radians members store radians.
    KDL::Rotation kdl_rotation = KDL::Rotation::RPY(pose.rx.value(), pose.ry.value(), pose.rz.value());
    // KDL Vector expects meters. Our RDT::Meters members store meters.
    KDL::Vector kdl_position(pose.x.value(), pose.y.value(), pose.z.value());
    KDL::Frame kdl_goal_frame(kdl_rotation, kdl_position);

    KDL::JntArray kdl_seed_jnt_array = toKdlJntArray(seed_joints);
    KDL::JntArray kdl_result_jnt_array(kdl_chain_ref_.getNrOfJoints());

    LOG_DEBUG_F(MODULE_NAME, "Attempting IK. Target Cart: %s, Seed Joint (deg): %s",
                pose.toDescriptiveString().c_str(), seed_joints.toJointPoseString().c_str());

    int ik_status = ik_pos_solver_->CartToJnt(kdl_seed_jnt_array, kdl_goal_frame, kdl_result_jnt_array);

    if (ik_status < 0) {
        LOG_WARN_F(MODULE_NAME, "KDL IK failed with error code: %d (Target: %s). Pose may be unreachable or near singularity.",
                   ik_status, pose.toDescriptiveString().c_str());
        return std::nullopt; // IK solution not found
    }

    AxisSet result_joints = fromKdlJntArray(kdl_result_jnt_array);
    LOG_DEBUG_F(MODULE_NAME, "IK success. Result Joint (deg): %s", result_joints.toJointPoseString().c_str());
    return result_joints;
}

void KdlKinematicSolver::setHomePosition(const AxisSet& home_joints) {
    if (home_joints.size() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_ERROR_F(MODULE_NAME, "Home position AxisSet size (%zu) does not match chain DOF (%u).", home_joints.size(), kdl_chain_ref_.getNrOfJoints());
        throw std::invalid_argument("KdlKinematicSolver::setHomePosition: Joint count mismatch.");
    }
    rdt_home_position_joints_ = home_joints;
    LOG_INFO_F(MODULE_NAME, "Home position set to (deg): %s", rdt_home_position_joints_.toJointPoseString().c_str());
}

AxisSet KdlKinematicSolver::getHomePosition() const {
    return rdt_home_position_joints_;
}

// --- Private Helper Methods ---
KDL::JntArray KdlKinematicSolver::toKdlJntArray(const AxisSet& rdt_joints) const {
    KDL::JntArray kdl_jnts(kdl_chain_ref_.getNrOfJoints());
    for (unsigned int i = 0; i < kdl_jnts.rows(); ++i) {
        // rdt_joints.at(i) returns RDT::Axis, .angle is RDT::Radians, .value() is double (radians)
        kdl_jnts(i) = rdt_joints.at(i).angle.value();
    }
    return kdl_jnts;
}

AxisSet KdlKinematicSolver::fromKdlJntArray(const KDL::JntArray& kdl_jnts) const {
    AxisSet rdt_axes; // Default constructs with RDT::ROBOT_AXES_COUNT axes
    if (kdl_jnts.rows() != rdt_axes.size()){
        LOG_ERROR_F(MODULE_NAME, "Size mismatch converting KDL::JntArray (%u) to AxisSet (%zu).", kdl_jnts.rows(), rdt_axes.size());
        // This should ideally not happen if KDL chain DOF matches ROBOT_AXES_COUNT
        throw std::length_error("KdlKinematicSolver::fromKdlJntArray: KDL JntArray size does not match AxisSet size.");
    }
    for (unsigned int i = 0; i < kdl_jnts.rows(); ++i) {
        // kdl_jnts(i) is double (radians), wrap it in RDT::Radians
        rdt_axes.at(i).angle = Radians(kdl_jnts(i));
        // Velocity and acceleration are not typically set by position IK/FK solvers
        rdt_axes.at(i).velocity = 0.0_rad_s;
        rdt_axes.at(i).acceleration = 0.0_rad_s2;
        // Other Axis fields (brake, servo) remain default unless KDL provides this info
    }
    return rdt_axes;
}

} // namespace RDT