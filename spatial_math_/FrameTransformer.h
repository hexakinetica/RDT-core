// FrameTransformer.h
#ifndef FRAMETRANSFORMER_H
#define FRAMETRANSFORMER_H

#pragma once

#include "DataTypes.h" // RDT::CartPose (  RDT::Meters, RDT::Radians)
#include "Units.h"     // RDT::Meters, RDT::Radians, RDT::literals (   )
#include "Logger.h"    // RDT::Logger (  ,     )

// Eigen     ,   CMakeLists.txt
// #undef Success  ,  Eigen    X11/wingdi 
#ifdef Success
#undef Success
#endif
#include <Eigen/Dense>    //  Eigen::Affine3d, Eigen::Vector3d
#include <Eigen/Geometry> //  Eigen::AngleAxisd

#include <string> //  MODULE_NAME_STATIC (  )

namespace RDT {

/**
 * @file FrameTransformer.h
 * @brief Defines the RDT::FrameTransformer class providing static utility methods
 *        for 3D Cartesian coordinate frame transformations.
 */

/**
 * @class FrameTransformer
 * @brief Provides static utility methods for 3D Cartesian coordinate frame transformations.
 *
 * This class offers a set of stateless functions to transform poses between different
 * coordinate frames (e.g., world, base, tool, local point relative to a frame) and
 * to combine or invert transformations. It uses Eigen for underlying matrix operations
 * and RDT strong types (CartPose, Meters, Radians) for its public interface.
 *
 * All transformations involving RDT::CartPose assume a ZYX Euler angle convention
 * (Yaw, Pitch, Roll corresponding to Rz, Ry, Rx respectively) when converting
 * to/from Eigen's rotation representations.
 */
class FrameTransformer {
public:
    // This class provides only static methods and should not be instantiated.
    FrameTransformer() = delete;
    ~FrameTransformer() = delete;
    FrameTransformer(const FrameTransformer&) = delete;
    FrameTransformer& operator=(const FrameTransformer&) = delete;
    FrameTransformer(FrameTransformer&&) = delete;
    FrameTransformer& operator=(FrameTransformer&&) = delete;

    /**
     * @brief Transforms a pose defined in a local coordinate system to the world coordinate system.
     *
     * This can be thought of as expressing a point/orientation P_local (defined relative to localFrame)
     * in world coordinates.
     * Formula: P_world = T_world_localFrame * P_localFrame
     *
     * @param pose_in_local_frame The pose (P_localFrame) of a point or sub-frame, defined relative to the localFrame.
     * @param local_frame_in_world The transformation (T_world_localFrame) that defines the localFrame's
     *                             origin and orientation with respect to the world frame.
     * @return CartPose The pose_in_local_frame now expressed in world coordinates (P_world).
     */
    [[nodiscard]] static CartPose transformPoseToWorld(const CartPose& pose_in_local_frame,
                                                       const CartPose& local_frame_in_world);

    /**
     * @brief Transforms a pose defined in the world coordinate system to a local coordinate system.
     *
     * This can be thought of as expressing a point/orientation P_world (defined in world coordinates)
     * relative to the localFrame.
     * Formula: P_localFrame = inv(T_world_localFrame) * P_world
     *
     * @param pose_in_world The pose (P_world) defined relative to the world frame.
     * @param local_frame_in_world The transformation (T_world_localFrame) that defines the localFrame's
     *                             origin and orientation with respect to the world frame.
     * @return CartPose The pose_in_world now expressed in the localFrame's coordinates (P_localFrame).
     */
    [[nodiscard]] static CartPose transformPoseFromWorld(const CartPose& pose_in_world,
                                                         const CartPose& local_frame_in_world);

    /**
     * @brief Calculates the Tool Center Point (TCP) pose in the world coordinate system.
     * Formula: T_world_tcp = T_world_flange * T_flange_tcp
     *
     * @param flange_pose_in_world The pose of the robot's flange in world coordinates (T_world_flange).
     * @param tool_transform_on_flange The transformation from the flange to the TCP (T_flange_tcp),
     *                                 typically part of an RDT::ToolFrame definition.
     * @return CartPose The pose of the TCP expressed in world coordinates (T_world_tcp).
     */
    [[nodiscard]] static CartPose calculateTcpInWorld(const CartPose& flange_pose_in_world,
                                                      const CartPose& tool_transform_on_flange);

    /**
     * @brief Calculates the robot's flange pose in the world coordinate system, given the TCP pose in world.
     * Formula: T_world_flange = T_world_tcp * inv(T_flange_tcp)
     *
     * @param tcp_pose_in_world The pose of the TCP in world coordinates (T_world_tcp).
     * @param tool_transform_on_flange The transformation from the flange to the TCP (T_flange_tcp).
     * @return CartPose The pose of the flange expressed in world coordinates (T_world_flange).
     */
    [[nodiscard]] static CartPose calculateFlangeInWorld(const CartPose& tcp_pose_in_world,
                                                         const CartPose& tool_transform_on_flange);

    /**
     * @brief Combines (multiplies) two transformations: T_world_A and T_A_B to get T_world_B.
     * This is equivalent to applying transformation T_A_B to an object already transformed by T_world_A.
     * Formula: T_world_B = T_world_A * T_A_B
     *
     * @param T_world_A The CartPose representing the transformation from World to frame A.
     * @param T_A_B The CartPose representing the transformation from frame A to frame B.
     * @return CartPose The resulting CartPose representing the transformation from World to frame B (T_world_B).
     */
    [[nodiscard]] static CartPose combineTransforms(const CartPose& T_world_A, const CartPose& T_A_B);

    /**
     * @brief Inverts a given transformation.
     * If the input `transform` represents T_A_B (transformation from frame B to frame A),
     * this method returns T_B_A (transformation from frame A to frame B).
     *
     * @param transform The CartPose representing the transformation T_A_B.
     * @return CartPose The inverted transformation T_B_A.
     */
    [[nodiscard]] static CartPose invertTransform(const CartPose& transform);

private:
    // Static helper methods for conversion between RDT::CartPose and Eigen::Affine3d
    // These are inline as they are simple and used frequently by the public static methods.

    /**
     * @internal
     * @brief Converts an RDT::CartPose (using RDT::Meters and RDT::Radians for ZYX Euler angles)
     *        to an Eigen::Affine3d transformation matrix.
     * @param p The RDT::CartPose to convert.
     * @return Eigen::Affine3d The equivalent Eigen transformation matrix.
     */
    [[nodiscard]] static inline Eigen::Affine3d poseToEigenAffine(const CartPose& p) {
        Eigen::Affine3d T = Eigen::Affine3d::Identity();
        T.translation() = Eigen::Vector3d(p.x.value(), p.y.value(), p.z.value());
        // Rotation: ZYX Euler: R = Rz * Ry * Rx
        T.linear() = (Eigen::AngleAxisd(p.rz.value(), Eigen::Vector3d::UnitZ()) *
                      Eigen::AngleAxisd(p.ry.value(), Eigen::Vector3d::UnitY()) *
                      Eigen::AngleAxisd(p.rx.value(), Eigen::Vector3d::UnitX())).toRotationMatrix();
        return T;
    }

    /**
     * @internal
     * @brief Converts an Eigen::Affine3d transformation matrix to an RDT::CartPose
     *        (populating RDT::Meters and RDT::Radians for ZYX Euler angles).
     * @param T The Eigen::Affine3d matrix to convert.
     * @return CartPose The equivalent RDT::CartPose.
     */
    [[nodiscard]] static inline CartPose eigenAffineToPose(const Eigen::Affine3d& T) {
        CartPose p;
        p.x = Meters(T.translation().x());
        p.y = Meters(T.translation().y());
        p.z = Meters(T.translation().z());
        // Eigen's eulerAngles(2, 1, 0) extracts Z(yaw), Y(pitch), X(roll)
        Eigen::Vector3d euler_rad = T.rotation().eulerAngles(2, 1, 0);
        p.rz = Radians(euler_rad[0]); // Yaw
        p.ry = Radians(euler_rad[1]); // Pitch
        p.rx = Radians(euler_rad[2]); // Roll
        return p;
    }
    
    // For logging within static methods, if uncommented in method bodies
    // static inline const std::string MODULE_NAME_STATIC = "FrameTransformer";
};


// --- Implementations of public static methods ---
// (Moved inline into class declaration for header-only, or can be here if preferred)

inline CartPose FrameTransformer::transformPoseToWorld(const CartPose& pose_in_local_frame,
                                                      const CartPose& local_frame_in_world) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "transformPoseToWorld: P_local=%s, T_world_local=%s", ...);
    Eigen::Affine3d T_world_local = poseToEigenAffine(local_frame_in_world);
    Eigen::Affine3d P_local = poseToEigenAffine(pose_in_local_frame);
    return eigenAffineToPose(T_world_local * P_local);
}

inline CartPose FrameTransformer::transformPoseFromWorld(const CartPose& pose_in_world,
                                                        const CartPose& local_frame_in_world) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "transformPoseFromWorld: P_world=%s, T_world_local=%s", ...);
    Eigen::Affine3d T_world_local = poseToEigenAffine(local_frame_in_world);
    Eigen::Affine3d P_world = poseToEigenAffine(pose_in_world);
    return eigenAffineToPose(T_world_local.inverse() * P_world);
}

inline CartPose FrameTransformer::calculateTcpInWorld(const CartPose& flange_pose_in_world,
                                                     const CartPose& tool_transform_on_flange) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "calculateTcpInWorld: Flange_w=%s, Tool_f=%s", ...);
    Eigen::Affine3d T_world_flange = poseToEigenAffine(flange_pose_in_world);
    Eigen::Affine3d T_flange_tcp = poseToEigenAffine(tool_transform_on_flange);
    return eigenAffineToPose(T_world_flange * T_flange_tcp);
}

inline CartPose FrameTransformer::calculateFlangeInWorld(const CartPose& tcp_pose_in_world,
                                                        const CartPose& tool_transform_on_flange) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "calculateFlangeInWorld: TCP_w=%s, Tool_f=%s", ...);
    Eigen::Affine3d T_world_tcp = poseToEigenAffine(tcp_pose_in_world);
    Eigen::Affine3d T_flange_tcp = poseToEigenAffine(tool_transform_on_flange);
    return eigenAffineToPose(T_world_tcp * T_flange_tcp.inverse());
}

inline CartPose FrameTransformer::combineTransforms(const CartPose& T_world_A, const CartPose& T_A_B) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "combineTransforms: T_w_A=%s, T_A_B=%s", ...);
    Eigen::Affine3d eigen_T_world_A = poseToEigenAffine(T_world_A);
    Eigen::Affine3d eigen_T_A_B = poseToEigenAffine(T_A_B);
    return eigenAffineToPose(eigen_T_world_A * eigen_T_A_B);
}

inline CartPose FrameTransformer::invertTransform(const CartPose& transform) {
    // LOG_DEBUG_F(MODULE_NAME_STATIC, "invertTransform: T_A_B=%s", ...);
    Eigen::Affine3d T_A_B = poseToEigenAffine(transform);
    return eigenAffineToPose(T_A_B.inverse());
}

} // namespace RDT
#endif // FRAMETRANSFORMER_H