// TrajectoryPlanner.h
#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#pragma once

#include "DataTypes.h"             // RDT types
#include "Units.h"                 // RDT units and literals
#include "KinematicSolver.h"       // RDT::KinematicSolver interface
#include "FrameTransformer.h"      // RDT::FrameTransformer (static methods)
#include "TrajectoryInterpolator.h"// RDT::TrajectoryInterpolator
#include "Logger.h"                // RDT::Logger

#include <deque>   // If used for internal point storage before interpolation window
#include <vector>  // For returning window of points
#include <memory>  // For std::shared_ptr
#include <string>  // For std::string
#include <optional>// For KinematicSolver results

namespace RDT {

/**
 * @file TrajectoryPlanner.h
 * @brief Defines the RDT::TrajectoryPlanner class for generating robot motion trajectories.
 */

/**
 * @class TrajectoryPlanner
 * @brief Plans and generates time-parameterized trajectory points between user-defined waypoints.
 *
 * The TrajectoryPlanner takes a sequence of target TrajectoryPoints, which can be defined
 * in various user coordinate systems and with different tools. It transforms these targets
 * into the robot's base coordinate system, then uses a TrajectoryInterpolator to generate
 * a smooth path between the current robot state and the transformed target.
 * For LIN motions, it performs Inverse Kinematics (IK) for each interpolated Cartesian point.
 * For JOINT/PTP motions, it performs Forward Kinematics (FK) for interpolated joint points
 * to provide corresponding Cartesian information.
 * The output is a window of TrajectoryPoints, primarily containing joint commands for a
 * lower-level motion controller, with all poses expressed in the robot's base frame
 * and assuming a "zero tool" (flange pose for Cartesian).
 */
class TrajectoryPlanner {
public:
    /**
     * @brief Constructs a TrajectoryPlanner.
     * @param solver A shared pointer to a KinematicSolver implementation.
     * @param interpolator A shared pointer to a TrajectoryInterpolator implementation.
     * @throws std::invalid_argument if solver or interpolator is null.
     */
    TrajectoryPlanner(std::shared_ptr<KinematicSolver> solver,
                      std::shared_ptr<TrajectoryInterpolator> interpolator);

    ~TrajectoryPlanner() = default;

    // Planner is stateful and manages complex dependencies, make non-copyable/movable.
    TrajectoryPlanner(const TrajectoryPlanner&) = delete;
    TrajectoryPlanner& operator=(const TrajectoryPlanner&) = delete;
    TrajectoryPlanner(TrajectoryPlanner&&) = delete;
    TrajectoryPlanner& operator=(TrajectoryPlanner&&) = delete;


    /**
     * @brief Sets the current state of the robot (pose in robot base frame).
     * This is used as the starting point for the next trajectory segment.
     * The provided current_robot_state.command should contain valid joint positions.
     * Its cartesian_target will be (re)calculated via FK if not consistent or if data_type implies.
     * The tool and base in current_robot_state are assumed to be identity/default for robot base frame.
     * @param current_robot_state The current state of the robot, primarily its joint positions.
     *                            All poses within are assumed to be in the robot's base frame.
     *                            Tool and Base in its header should be default (identity).
     */
    void setCurrentRobotState(const TrajectoryPoint& current_robot_state);

    /**
     * @brief Adds the next target TrajectoryPoint to be planned.
     * The target point's pose_cart is defined in the coordinate system specified by its Header.base,
     * and it represents the TCP pose defined by its Header.tool.
     * The planner transforms this target into the robot's base coordinate system before processing.
     * @param next_target_waypoint The target waypoint, potentially in a user frame.
     * @return true if the target was successfully added and the segment to it is loaded for interpolation.
     *         Returns false if the planner is already busy with a segment, if the current robot state
     *         is not set, or if an error occurs during transformation or segment loading (e.g., IK failure for LIN start).
     */
    [[nodiscard]] bool addTargetWaypoint(const TrajectoryPoint& next_target_waypoint);

    /**
     * @brief Retrieves a window of interpolated TrajectoryPoints for the current segment.
     * The returned points are in the robot's base coordinate system.
     * For JOINT/PTP types, point.command.joint_target is primary, point.command.cartesian_target (flange) is FK-derived.
     * For LIN types, point.command.joint_target is primary (IK-derived), point.command.cartesian_target (flange) is from FK(IK).
     * Header.tool and Header.base in output points are set to default (identity).
     *
     * @param dt_sample The time step for generating points within the window.
     * @param window_duration The total duration of the window to generate.
     * @return std::vector<TrajectoryPoint> A vector of interpolated points. Empty if segment done or error.
     */
    [[nodiscard]] std::vector<TrajectoryPoint> getNextPointWindow(Seconds dt_sample, Seconds window_duration);

    /**
     * @brief Checks if the current trajectory segment has been fully interpolated.
     * @return true if the current segment is done or if no segment is active, or if an error occurred.
     */
    [[nodiscard]] bool isCurrentSegmentDone() const;

    /**
     * @brief Checks if the planner is in an error state.
     * @return true if an error has occurred, false otherwise.
     */
    [[nodiscard]] bool hasError() const;

    /**
     * @brief Gets the last error message.
     * @return std::string The error message. Empty if no error.
     */
    [[nodiscard]] std::string getErrorMessage() const;
    
        /** @internal @brief Clears the error state. Made public for RobotController. */
    void clearError();
    
private:
    /**
     * @internal
     * @brief Transforms a waypoint from its defined user/tool frame to a target for the robot's flange in the robot's base frame.
     * For JOINT/PTP: joint_target is preserved, cartesian_target (flange) is FK from joints.
     * For LIN: cartesian_target (TCP) is transformed to base frame, then to flange target in base frame. Joint target is cleared.
     * @param waypoint_in_user_frame The waypoint to transform.
     * @return TrajectoryPoint The transformed waypoint, with poses ready for IK/FK relative to robot base,
     *         and its Header.tool and Header.base set to default.
     * @throws std::runtime_error if transformation fails.
     */
    [[nodiscard]] TrajectoryPoint transformWaypointToRobotBaseFlangeTarget(const TrajectoryPoint& waypoint_in_user_frame);

    /**
     * @internal
     * @brief Loads the segment into the interpolator after all transformations and IK/FK for start/end points.
     * @param start_point_base_flange Start of segment (flange pose in base).
     * @param end_point_base_flange End of segment (flange pose in base).
     * @return true on success.
     */
    bool loadSegmentForInterpolator(const TrajectoryPoint& start_point_base_flange,
                                    const TrajectoryPoint& end_point_base_flange);
    
    /** @internal @brief Sets an error state. */
    void setError(const std::string& message);
   // /** @internal @brief Clears the error state. */
   // void clearError();

    std::shared_ptr<KinematicSolver> solver_;
    std::shared_ptr<TrajectoryInterpolator> interpolator_;
    // FrameTransformer is header-only, no need for shared_ptr if only static methods are used.
    // If an instance was needed: std::shared_ptr<FrameTransformer> transformer_;

    TrajectoryPoint current_robot_state_base_flange_; ///< Current robot state (flange pose in base frame).
    bool current_state_is_set_ = false;            ///< True if setCurrentRobotState has been called successfully.
    bool segment_is_active_ = false;               ///< True if a segment is loaded in the interpolator.

    // Error state
    bool error_flag_ = false;
    std::string error_message_;

    // Seed for IK, always in robot base frame joint values.
    AxisSet last_ik_seed_joints_;

    // Store the target waypoint as defined by the user (before transformation)
    // This helps retain original tool/base/speed for metadata in interpolated points.
    TrajectoryPoint current_segment_user_target_waypoint_;

    static inline const std::string MODULE_NAME = "TrajPlanner";
};

} // namespace RDT
#endif // TRAJECTORYPLANNER_H
