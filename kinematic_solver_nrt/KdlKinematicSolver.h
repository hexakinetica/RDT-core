// KdlKinematicSolver.h
#ifndef KDLKINEMATICSOLVER_H
#define KDLKINEMATICSOLVER_H

#pragma once

#include "KinematicSolver.h" // Теперь содержит IKResult и IKStatus
#include "KinematicModel.h"  
#include "DataTypes.h"       
#include "Units.h"           
#include "Logger.h"          

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>   
#include <kdl/jntarray.hpp>
#include <kdl/frames.hpp>  

#include <memory>   
#include <atomic> 

#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_lma.hpp>

namespace RDT {

class KdlKinematicSolver : public KinematicSolver {
public:
    explicit KdlKinematicSolver(const KinematicModel& model);

    // Сигнатуры теперь совпадают с базовым классом
    [[nodiscard]] bool solveFK(const AxisSet& joints, CartPose& result) const override;
    
    [[nodiscard]] IKResult solveIK(const CartPose& pose,
                                   const AxisSet& seed_joints,
                                   const IKHint& hint = {}) const override;

    void setHomePosition(const AxisSet& home_joints) override;
    [[nodiscard]] AxisSet getHomePosition() const override;

    void setFallbackEnabled(bool enabled);
    [[nodiscard]] bool isFallbackEnabled() const;

private:
    const KDL::Chain& kdl_chain_ref_; 
    KDL::JntArray kdl_joint_min_copy_; 
    KDL::JntArray kdl_joint_max_copy_;
    AxisSet rdt_home_position_joints_; 

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_wdls> ik_vel_solver_; 
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_; 
    std::unique_ptr<KDL::ChainIkSolverPos_LMA> ik_pos_solver_fallback_;

    KDL::JntArray weights_joints_;
    KDL::Twist    weights_cart_;

    std::atomic<bool> fallback_enabled_{true};

    [[nodiscard]] KDL::JntArray toKdlJntArray(const AxisSet& rdt_joints) const;
    [[nodiscard]] AxisSet fromKdlJntArray(const KDL::JntArray& kdl_jnts) const;
    void clampToJointLimits(KDL::JntArray& joints) const;

    static inline const std::string MODULE_NAME = "KdlSolver"; 
};

} // namespace RDT
#endif // KDLKINEMATICSOLVER_H