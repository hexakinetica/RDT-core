// KdlKinematicSolver.cpp
#include "KdlKinematicSolver.h"
#include <algorithm> // For std::min/max

namespace RDT {

using namespace RDT::literals;

KdlKinematicSolver::KdlKinematicSolver(const KinematicModel& model)
    : kdl_chain_ref_(model.getChain()),
      kdl_joint_min_copy_(model.getKdlJointMinLimits()),
      kdl_joint_max_copy_(model.getKdlJointMaxLimits()),
      rdt_home_position_joints_(model.getHomePositionJoints()) {

    LOG_INFO_F(MODULE_NAME, "Initializing KDL Kinematic Solver for a chain with %u joints.", kdl_chain_ref_.getNrOfJoints());
    if (kdl_chain_ref_.getNrOfJoints() == 0) { throw std::logic_error("KdlKinematicSolver: KDL chain is empty."); }
    if (kdl_joint_min_copy_.rows() != kdl_chain_ref_.getNrOfJoints() || kdl_joint_max_copy_.rows() != kdl_chain_ref_.getNrOfJoints()) { throw std::logic_error("KdlKinematicSolver: Joint limit size mismatch."); }
    
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(kdl_chain_ref_);
    if (!fk_solver_) { throw std::bad_alloc(); }

    weights_joints_.resize(kdl_chain_ref_.getNrOfJoints());
    for(unsigned int i=0; i < kdl_chain_ref_.getNrOfJoints(); i++) { weights_joints_(i) = 1.0; }
    weights_cart_.vel.x(1.0); weights_cart_.vel.y(1.0); weights_cart_.vel.z(1.0);
    weights_cart_.rot.x(0.01); weights_cart_.rot.y(0.01); weights_cart_.rot.z(0.01);
    
    ik_vel_solver_ = std::make_unique<KDL::ChainIkSolverVel_wdls>(kdl_chain_ref_);
    if (!ik_vel_solver_) { throw std::bad_alloc(); }
    // Convert KDL::JntArray to Eigen::MatrixXd for setWeightJS
    Eigen::MatrixXd weights_joints_eigen = Eigen::MatrixXd::Identity(kdl_chain_ref_.getNrOfJoints(), kdl_chain_ref_.getNrOfJoints());
    for(unsigned int i=0; i < kdl_chain_ref_.getNrOfJoints(); i++) {
        weights_joints_eigen(i, i) = weights_joints_(i);
    }
    ik_vel_solver_->setWeightJS(weights_joints_eigen);

    // Convert KDL::Twist (weights_cart_) to Eigen::MatrixXd for setWeightTS
    Eigen::MatrixXd weights_cart_eigen = Eigen::MatrixXd::Zero(6, 6);
    weights_cart_eigen(0, 0) = weights_cart_.vel.x();
    weights_cart_eigen(1, 1) = weights_cart_.vel.y();
    weights_cart_eigen(2, 2) = weights_cart_.vel.z();
    weights_cart_eigen(3, 3) = weights_cart_.rot.x();
    weights_cart_eigen(4, 4) = weights_cart_.rot.y();
    weights_cart_eigen(5, 5) = weights_cart_.rot.z();

    ik_vel_solver_->setWeightTS(weights_cart_eigen);   
    ik_vel_solver_->setLambda(1e-2); // Regularization factor
    
    ik_pos_solver_ = std::make_unique<KDL::ChainIkSolverPos_NR_JL>(
        kdl_chain_ref_, kdl_joint_min_copy_, kdl_joint_max_copy_,
        *fk_solver_, *ik_vel_solver_, 300, 1e-4);
    if (!ik_pos_solver_) { throw std::bad_alloc(); }

    ik_pos_solver_fallback_ = std::make_unique<KDL::ChainIkSolverPos_LMA>(kdl_chain_ref_, 1e-3, 300, 1e-5);
    if (!ik_pos_solver_fallback_) { throw std::bad_alloc(); }
    
    LOG_INFO(MODULE_NAME, "KDL Solvers initialized (Primary: Weighted NR_JL, Fallback: LMA).");
}

bool KdlKinematicSolver::solveFK(const AxisSet& joints, CartPose& result) const {
    if (joints.size() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_ERROR_F(MODULE_NAME, "FK input AxisSet size (%zu) does not match chain DOF (%u).", joints.size(), kdl_chain_ref_.getNrOfJoints());
        return false;
    }
    KDL::JntArray kdl_jnt_array = toKdlJntArray(joints);
    KDL::Frame kdl_frame_result;

    int fk_status = fk_solver_->JntToCart(kdl_jnt_array, kdl_frame_result);
    if (fk_status < 0) {
        LOG_ERROR_F(MODULE_NAME, "KDL FK computation failed with error code: %d", fk_status);
        return false;
    }
    
    result.x = Meters(kdl_frame_result.p.x());
    result.y = Meters(kdl_frame_result.p.y());
    result.z = Meters(kdl_frame_result.p.z());
    
    double kdl_r, kdl_p, kdl_y;
    kdl_frame_result.M.GetRPY(kdl_r, kdl_p, kdl_y);
    result.rx = Radians(kdl_r);
    result.ry = Radians(kdl_p);
    result.rz = Radians(kdl_y);

    LOG_DEBUG_F(MODULE_NAME, "FK solved. Joint: %s -> Cart: %s",
                joints.toJointPoseString().c_str(), result.toDescriptiveString().c_str());
    return true;
}

IKResult KdlKinematicSolver::solveIK(const CartPose& pose,
                                     const AxisSet& seed_joints,
                                     const IKHint& hint) const {
    (void)hint; 
    
    if (seed_joints.size() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_ERROR_F(MODULE_NAME, "IK input seed_joints size (%zu) does not match chain DOF (%u).", seed_joints.size(), kdl_chain_ref_.getNrOfJoints());
        return {seed_joints, IKStatus::Failed};
    }

    KDL::Rotation kdl_rotation = KDL::Rotation::RPY(pose.rx.value(), pose.ry.value(), pose.rz.value());
    KDL::Vector kdl_position(pose.x.value(), pose.y.value(), pose.z.value());
    KDL::Frame kdl_goal_frame(kdl_rotation, kdl_position);

    KDL::JntArray kdl_seed_jnt_array = toKdlJntArray(seed_joints);
    KDL::JntArray kdl_result_jnt_array(kdl_chain_ref_.getNrOfJoints());

    // --- Попытка №1: Основной решатель ---
    LOG_DEBUG(MODULE_NAME, "Attempting IK with primary solver (Weighted NR_JL)...");
    int ik_status = ik_pos_solver_->CartToJnt(kdl_seed_jnt_array, kdl_goal_frame, kdl_result_jnt_array);

    if (ik_status >= 0) {
        LOG_DEBUG(MODULE_NAME, "IK success with primary solver.");
        return {fromKdlJntArray(kdl_result_jnt_array), IKStatus::Precise};
    }

    // --- Проверка, разрешен ли запасной решатель ---
    if (!fallback_enabled_.load()) {
        LOG_WARN_F(MODULE_NAME, "Primary IK solver failed with code %d. Fallback is disabled.", ik_status);
        return {seed_joints, IKStatus::Failed};
    }

    // --- Попытка №2: Запасной решатель ---
   // LOG_WARN_F(MODULE_NAME, "Primary IK solver failed with code %d. Trying fallback solver (LMA)...", ik_status);
    
    ik_status = ik_pos_solver_fallback_->CartToJnt(kdl_seed_jnt_array, kdl_goal_frame, kdl_result_jnt_array);
    
    if (ik_status >= 0) {
        // ИСПРАВЛЕНИЕ: Макрос логгера требует 2 аргумента
     //   LOG_WARN(MODULE_NAME, "IK success with fallback solver. Clamping result to joint limits.");
        clampToJointLimits(kdl_result_jnt_array);
        return {fromKdlJntArray(kdl_result_jnt_array), IKStatus::Approximate};
    }

    // --- Провал ---
    LOG_ERROR_F(MODULE_NAME, "All IK solvers failed. Pose is likely unreachable: %s", pose.toDescriptiveString().c_str());
    return {seed_joints, IKStatus::Failed};
}

void KdlKinematicSolver::setFallbackEnabled(bool enabled) {
    fallback_enabled_.store(enabled);
    LOG_INFO_F(MODULE_NAME, "IK fallback solver has been %s.", enabled ? "ENABLED" : "DISABLED");
}

bool KdlKinematicSolver::isFallbackEnabled() const {
    return fallback_enabled_.load();
}


void KdlKinematicSolver::clampToJointLimits(KDL::JntArray& joints) const {
    for (unsigned int i = 0; i < joints.rows(); ++i) {
        joints(i) = std::max(kdl_joint_min_copy_(i), std::min(joints(i), kdl_joint_max_copy_(i)));
    }
}

void KdlKinematicSolver::setHomePosition(const AxisSet& home_joints) {
    if (home_joints.size() != kdl_chain_ref_.getNrOfJoints()) {
        LOG_CRITICAL_F(MODULE_NAME, "Home position AxisSet size (%zu) does not match chain DOF (%u).", home_joints.size(), kdl_chain_ref_.getNrOfJoints());
        return;
    }
    rdt_home_position_joints_ = home_joints;
    LOG_INFO_F(MODULE_NAME, "Home position set to (deg): %s", rdt_home_position_joints_.toJointPoseString().c_str());
}

AxisSet KdlKinematicSolver::getHomePosition() const {
    return rdt_home_position_joints_;
}

KDL::JntArray KdlKinematicSolver::toKdlJntArray(const AxisSet& rdt_joints) const {
    KDL::JntArray kdl_jnts(kdl_chain_ref_.getNrOfJoints());
    for (unsigned int i = 0; i < kdl_jnts.rows(); ++i) {
        kdl_jnts(i) = rdt_joints.at(i).angle.value();
    }
    return kdl_jnts;
}

AxisSet KdlKinematicSolver::fromKdlJntArray(const KDL::JntArray& kdl_jnts) const {
    AxisSet rdt_axes; 
    if (kdl_jnts.rows() != rdt_axes.size()){
        LOG_CRITICAL_F(MODULE_NAME, "Size mismatch converting KDL::JntArray (%u) to AxisSet (%zu).", kdl_jnts.rows(), rdt_axes.size());
        throw std::length_error("KdlKinematicSolver::fromKdlJntArray: KDL JntArray size does not match AxisSet size.");
    }
    for (unsigned int i = 0; i < kdl_jnts.rows(); ++i) {
        rdt_axes.at(i).angle = Radians(kdl_jnts(i));
        rdt_axes.at(i).velocity = 0.0_rad_s;
        rdt_axes.at(i).acceleration = 0.0_rad_s2;
    }
    return rdt_axes;
}

} // namespace RDT