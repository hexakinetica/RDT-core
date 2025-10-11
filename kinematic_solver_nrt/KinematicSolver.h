// KinematicSolver.h
#ifndef KINEMATICSOLVER_H
#define KINEMATICSOLVER_H

#pragma once

#include "DataTypes.h" 
#include <string>      

namespace RDT {

// Переносим IKResult и IKStatus сюда, так как это часть интерфейса
enum class IKStatus {
    Precise,
    Approximate,
    Failed
};

struct IKResult {
    AxisSet solution;
    IKStatus status;
};

struct IKHint {
    std::string custom_solver_flags;
};

class KinematicSolver {
public:
    virtual ~KinematicSolver() = default;

    /**
     * @brief Solves the Forward Kinematics problem.
     * @param joints The set of joint angles.
     * @param[out] result The calculated Cartesian pose is written here on success.
     * @return true if FK computation was successful, false otherwise.
     */
    [[nodiscard]] virtual bool solveFK(const AxisSet& joints, CartPose& result) const = 0;

    /**
     * @brief Solves the Inverse Kinematics problem.
     * @param pose The desired Cartesian pose.
     * @param seed_joints An initial guess or current joint positions.
     * @param hint Optional hints.
     * @return IKResult structure containing the solution and a status code.
     *         The caller MUST check result.status before using result.solution.
     */
    [[nodiscard]] virtual IKResult solveIK(const CartPose& pose,
                                           const AxisSet& seed_joints,
                                           const IKHint& hint = {}) const = 0;

    virtual void setHomePosition(const AxisSet& home_joints) = 0;
    [[nodiscard]] virtual AxisSet getHomePosition() const = 0;
};

} // namespace RDT
#endif // KINEMATICSOLVER_H