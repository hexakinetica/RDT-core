// KinematicSolver.h
#ifndef KINEMATICSOLVER_H
#define KINEMATICSOLVER_H

#pragma once

#include "DataTypes.h" // RDT::CartPose, RDT::AxisSet
#include <optional>    // For std::optional in solveIK return type
#include <string>      // For std::string if needed in future hints
#include <stdexcept>   // For std::runtime_error (used by implementations)

namespace RDT {

/**
 * @struct IKHint
 * @brief Provides hints or constraints for the Inverse Kinematics (IK) solver.
 * These hints can guide the solver to find a more desirable solution among multiple possibilities.
 */
struct IKHint {
    // bool prefer_elbow_up = true;     // Example: KUKA-like configuration
    // bool prefer_positive_rz = true;  // Example: Prefer non-flipped wrist
    // Add specific hints relevant to your IK solver or robot configurations if needed.
    // For KDL's ChainIkSolverPos_NR_JL, direct configuration flags are limited.
    // More complex hint systems would require custom IK logic or different solvers.
    std::string custom_solver_flags; ///< Placeholder for future solver-specific flags.
};

/**
 * @class KinematicSolver
 * @brief Interface for Forward and Inverse Kinematics solvers.
 *
 * This abstract class defines the contract for converting between joint space
 * configurations (AxisSet) and Cartesian space poses (CartPose).
 */
class KinematicSolver {
public:
    virtual ~KinematicSolver() = default;

    /**
     * @brief Solves the Forward Kinematics problem.
     * Converts joint positions to a Cartesian pose of the end-effector.
     * @param joints The set of joint angles (AxisSet).
     * @return CartPose The calculated Cartesian pose of the end-effector.
     * @throws std::runtime_error if FK computation fails.
     */
    [[nodiscard]] virtual CartPose solveFK(const AxisSet& joints) const = 0;

    /**
     * @brief Solves the Inverse Kinematics problem.
     * Converts a desired Cartesian pose of the end-effector to a set of joint positions.
     * @param pose The desired Cartesian pose (CartPose).
     * @param seed_joints An initial guess or current joint positions (AxisSet) to help the solver converge.
     * @param hint Optional hints to guide the IK solution (IKHint).
     * @return std::optional<AxisSet> The calculated joint positions if a solution is found,
     *         std::nullopt otherwise (e.g., pose unreachable, singularity, solver did not converge).
     */
    [[nodiscard]] virtual std::optional<AxisSet> solveIK(const CartPose& pose,
                                                         const AxisSet& seed_joints,
                                                         const IKHint& hint = {}) const = 0;

    /**
     * @brief Sets the pre-defined home position for the robot in joint space.
     * @param home_joints The AxisSet representing the home position.
     */
    virtual void setHomePosition(const AxisSet& home_joints) = 0;

    /**
     * @brief Gets the pre-defined home position of the robot in joint space.
     * @return AxisSet The current home position.
     */
    [[nodiscard]] virtual AxisSet getHomePosition() const = 0;
};

} // namespace RDT
#endif // KINEMATICSOLVER_H