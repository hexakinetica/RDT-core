// KdlKinematicSolver.h
#ifndef KDLKINEMATICSOLVER_H
#define KDLKINEMATICSOLVER_H

#pragma once

#include "KinematicSolver.h" // RDT::KinematicSolver, RDT::IKHint
#include "KinematicModel.h"  // RDT::KinematicModel
#include "DataTypes.h"       // RDT::CartPose, RDT::AxisSet
#include "Units.h"           // RDT::Radians, RDT::Meters
#include "Logger.h"          // RDT::Logger

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>
#include <kdl/chain.hpp>   // For KDL::Chain
#include <kdl/jntarray.hpp>// For KDL::JntArray
#include <kdl/frames.hpp>  // For KDL::Frame

#include <memory>   // For std::unique_ptr
#include <optional> // For std::optional in solveIK

namespace RDT {

/**
 * @class KdlKinematicSolver
 * @brief Implements the KinematicSolver interface using the Kinematics and Dynamics Library (KDL).
 *
 * This class provides Forward Kinematics (FK) and Inverse Kinematics (IK) solutions
 * based on a given RDT::KinematicModel which encapsulates a KDL::Chain.
 */
class KdlKinematicSolver : public KinematicSolver {
public:
    /**
     * @brief Constructs a KdlKinematicSolver for the given robot model.
     * Initializes KDL solvers (FK, IK velocity, IK position with joint limits).
     * @param model The KinematicModel of the robot.
     * @throws std::runtime_error if KDL solver initialization fails.
     */
    explicit KdlKinematicSolver(const KinematicModel& model);

    // Override virtual methods from KinematicSolver
    [[nodiscard]] CartPose solveFK(const AxisSet& joints) const override;
    [[nodiscard]] std::optional<AxisSet> solveIK(const CartPose& pose,
                                                 const AxisSet& seed_joints,
                                                 const IKHint& hint = {}) const override;

    void setHomePosition(const AxisSet& home_joints) override;
    [[nodiscard]] AxisSet getHomePosition() const override;

private:
    // KDL specific members
    const KDL::Chain& kdl_chain_ref_; // Reference to the chain in the model
    // KDL::JntArray are passed by value to solver, so copy or direct access to model's
    // const KDL::JntArray& kdl_joint_min_ref_;
    // const KDL::JntArray& kdl_joint_max_ref_;
    // For ChainIkSolverPos_NR_JL, it takes JntArrays by value in constructor, so store copies or ensure lifetime
    KDL::JntArray kdl_joint_min_copy_; // Store copies of limits for solver
    KDL::JntArray kdl_joint_max_copy_;

    AxisSet rdt_home_position_joints_; // Store home position in RDT format

    std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
    std::unique_ptr<KDL::ChainIkSolverVel_pinv> ik_vel_solver_; // Used by pos IK solver
    std::unique_ptr<KDL::ChainIkSolverPos_NR_JL> ik_pos_solver_; // Position IK solver

    // Helper methods for converting between RDT::AxisSet and KDL::JntArray
    [[nodiscard]] KDL::JntArray toKdlJntArray(const AxisSet& rdt_joints) const;
    [[nodiscard]] AxisSet fromKdlJntArray(const KDL::JntArray& kdl_jnts) const;

    static inline const std::string MODULE_NAME = "KdlSolver"; // For logging
};

} // namespace RDT
#endif // KDLKINEMATICSOLVER_H