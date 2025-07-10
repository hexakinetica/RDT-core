// StateData.h
#ifndef STATEDATA_H
#define STATEDATA_H

#pragma once

#include "DataTypes.h" // RDT::TrajectoryPoint, ToolFrame, BaseFrame, AxisSet, RobotMode
                      //     Units.h,   RDT::literals .
#include "Logger.h"    // RDT::Logger     (  StateData )

#include <mutex>          //  std::unique_lock
#include <shared_mutex>   //  std::shared_mutex, std::shared_lock
#include <string>         //  std::string
#include <atomic>         //  std::atomic,        

namespace RDT {

/**
 * @file StateData.h
 * @brief Defines the RDT::StateData class for thread-safe storage and exchange
 *        of robot and system state information.
 */

/**
 * @class StateData
 * @brief A thread-safe container for sharing current robot state and operational data.
 *
 * This class provides synchronized access to various pieces of information, such as
 * command and feedback poses, active tool/base frames, robot mode, error messages, etc.
 * It uses std::shared_mutex for read-write locking, allowing multiple concurrent readers
 * or a single writer for each logical group of data.
 *
 * It is intended to be used as a central point for data exchange between different
 * components of the robot control system, like a RobotController and a GUI.
 */
class StateData {
public:
    /**
     * @brief Default constructor. Initializes all state data to default values.
     * For example, poses are default-constructed (often zero), mode is Idle, etc.
     */
    StateData()
        : robot_mode_(RobotMode::Idle),
          has_error_(false),
          speed_ratio_(1.0), // Default to 100% speed (1.0 ratio)
          estop_active_(false),
          is_physically_connected_(false) {
        // TrajectoryPoint, ToolFrame, BaseFrame, AxisSet members
        // (current_cmd_pose_, current_fb_pose_, active_tool_, active_base_frame_, actual_joint_state_)
        // are default-constructed by their respective default constructors.
        LOG_DEBUG("StateData", "StateData object created and initialized to defaults.");
    }

    // StateData is typically unique per application or major component,
    // and managing its lifetime via shared_ptr or direct ownership is preferred.
    // Making it non-copyable and non-movable by default is a safe choice
    // due to mutexes and complex state.
    StateData(const StateData&) = delete;
    StateData& operator=(const StateData&) = delete;
    StateData(StateData&&) = delete; // Could be implemented if carefully managed
    StateData& operator=(StateData&&) = delete; // Could be implemented

    // --- Commanded Pose (target for current motion segment) ---
    /**
     * @brief Sets the current commanded TrajectoryPoint.
     * This typically represents the target the robot is currently moving towards
     * or the last point commanded by the TrajectoryPlanner to the MotionManager.
     * @param cmd_point The TrajectoryPoint containing command data.
     */
    void setCmdTrajectoryPoint(const TrajectoryPoint& cmd_point) {
        std::unique_lock<std::shared_mutex> lock(mutex_cmd_pose_);
        current_cmd_pose_ = cmd_point;
    }

    /** @return A copy of the current commanded TrajectoryPoint. */
    [[nodiscard]] TrajectoryPoint getCmdTrajectoryPoint() const {
        std::shared_lock<std::shared_mutex> lock(mutex_cmd_pose_);
        return current_cmd_pose_;
    }

    // --- Feedback Pose (actual robot state) ---
    /**
     * @brief Sets the current feedback TrajectoryPoint.
     * This represents the latest reported state from the robot/motion system.
     * @param fb_point The TrajectoryPoint containing feedback data.
     */
    void setFbTrajectoryPoint(const TrajectoryPoint& fb_point) {
        std::unique_lock<std::shared_mutex> lock(mutex_fb_pose_);
        current_fb_pose_ = fb_point;
    }

    /** @return A copy of the current feedback TrajectoryPoint. */
    [[nodiscard]] TrajectoryPoint getFbTrajectoryPoint() const {
        std::shared_lock<std::shared_mutex> lock(mutex_fb_pose_);
        return current_fb_pose_;
    }

    // --- Active Tool ---
    /** @brief Sets the currently active tool frame. */
    void setActiveToolFrame(const ToolFrame& tool) {
        std::unique_lock<std::shared_mutex> lock(mutex_active_tool_);
        active_tool_ = tool;
    }
    /** @return A copy of the currently active tool frame. */
    [[nodiscard]] ToolFrame getActiveToolFrame() const {
        std::shared_lock<std::shared_mutex> lock(mutex_active_tool_);
        return active_tool_;
    }

    // --- Active Base Frame (User Frame) ---
    /** @brief Sets the currently active base coordinate frame. */
    void setActiveBaseFrame(const BaseFrame& base) {
        std::unique_lock<std::shared_mutex> lock(mutex_active_base_frame_);
        active_base_frame_ = base;
    }
    /** @return A copy of the currently active base coordinate frame. */
    [[nodiscard]] BaseFrame getActiveBaseFrame() const {
        std::shared_lock<std::shared_mutex> lock(mutex_active_base_frame_);
        return active_base_frame_;
    }

    // --- Robot Operational Mode ---
    /** @brief Sets the robot's current operational mode. */
    void setRobotMode(RobotMode mode) {
        // For RobotMode, which is a small enum, std::atomic might be an alternative
        // to std::shared_mutex if contention is high and updates are frequent/simple.
        // However, shared_mutex is fine for consistency.
        std::unique_lock<std::shared_mutex> lock(mutex_robot_mode_);
        robot_mode_ = mode;
    }
    /** @return The robot's current operational mode. */
    [[nodiscard]] RobotMode getRobotMode() const {
        std::shared_lock<std::shared_mutex> lock(mutex_robot_mode_);
        return robot_mode_;
    }

    // --- Actual Joint State (often part of FbTrajectoryPoint, but can be separate) ---
    /** @brief Sets the actual joint state of the robot. */
    void setActualJointState(const AxisSet& joints) {
        std::unique_lock<std::shared_mutex> lock(mutex_actual_joint_state_);
        actual_joint_state_ = joints;
    }
    /** @return A copy of the actual joint state. */
    [[nodiscard]] AxisSet getActualJointState() const {
        std::shared_lock<std::shared_mutex> lock(mutex_actual_joint_state_);
        return actual_joint_state_;
    }

    // --- Error Message and Status ---
    /**
     * @brief Sets an error message and status.
     * @param msg The error message string.
     * @param is_active_error True if this message represents an active error, false if clearing error or just a status.
     */
    void setSystemMessage(const std::string& msg, bool is_active_error) {
        std::unique_lock<std::shared_mutex> lock(mutex_system_message_);
        system_message_ = msg;
        has_error_ = is_active_error;
    }
    /** @return The current system/error message. */
    [[nodiscard]] std::string getSystemMessage() const {
        std::shared_lock<std::shared_mutex> lock(mutex_system_message_);
        return system_message_;
    }
    /** @return True if there is an active error associated with the current system message. */
    [[nodiscard]] bool hasActiveError() const {
        std::shared_lock<std::shared_mutex> lock(mutex_system_message_);
        return has_error_;
    }

    // --- Global Speed Ratio (Override) ---
    /**
     * @brief Sets the global speed override as a ratio (0.0 to 1.0).
     * @param ratio Speed ratio (e.g., 1.0 for 100%, 0.5 for 50%). Clamped to [0,1].
     */
    void setGlobalSpeedRatio(double ratio) {
        std::unique_lock<std::shared_mutex> lock(mutex_speed_ratio_);
        speed_ratio_ = std::max(0.0, std::min(1.0, ratio)); // Clamp ratio
    }
    /** @return The current global speed override ratio. */
    [[nodiscard]] double getGlobalSpeedRatio() const {
        std::shared_lock<std::shared_mutex> lock(mutex_speed_ratio_);
        return speed_ratio_;
    }

    // --- E-Stop State ---
    /**
     * @brief Sets the E-Stop state.
     * If E-Stop is activated, RobotMode should also reflect this.
     * @param is_estopped True if E-Stop is active.
     */
    void setEstopState(bool is_estopped) {
        std::unique_lock<std::shared_mutex> lock_estop(mutex_estop_state_);
        estop_active_ = is_estopped;
        // Also update RobotMode if E-Stop is activated.
        // This requires acquiring another mutex; consider order or combined state.
        // For simplicity now, assume setRobotMode will be called separately by controller if needed,
        // or make a combined method.
        // A better way: setEstopState updates estop_active_, and RobotController reacts to this
        // change by setting RobotMode.
        // Or, RobotMode::EStop is the single source of truth.
        // Let's assume EStop is a primary state overriding RobotMode for now.
        if (is_estopped) {
            std::unique_lock<std::shared_mutex> lock_mode(mutex_robot_mode_);
            robot_mode_ = RobotMode::EStop;
        }
    }
    /** @return True if E-Stop is currently active. */
    [[nodiscard]] bool isEstopActive() const {
        std::shared_lock<std::shared_mutex> lock(mutex_estop_state_);
        return estop_active_;
    }

    // --- Physical Connection Status (e.g., to real robot hardware) ---
    /** @brief Sets the status of physical connection to the robot hardware. */
    void setPhysicalConnectionStatus(bool connected) {
        std::unique_lock<std::shared_mutex> lock(mutex_connection_status_);
        is_physically_connected_ = connected;
    }
    /** @return True if a physical connection to robot hardware is established. */
    [[nodiscard]] bool isPhysicallyConnected() const {
        std::shared_lock<std::shared_mutex> lock(mutex_connection_status_);
        return is_physically_connected_;
    }

private:
    // --- Data Members (using RDT types) ---
    TrajectoryPoint current_cmd_pose_;    ///< Commanded/Target TrajectoryPoint for current motion.
    TrajectoryPoint current_fb_pose_;     ///< Feedback TrajectoryPoint representing actual robot state.
    ToolFrame active_tool_;               ///< Currently active tool.
    BaseFrame active_base_frame_;         ///< Currently active base/user coordinate frame.
    RobotMode robot_mode_;                ///< Overall operational mode of the robot.
    AxisSet actual_joint_state_;          ///< Separate actual joint state (can be redundant with current_fb_pose_).

    std::string system_message_;          ///< General status or error message.
    bool has_error_;                      ///< Flag indicating if system_message_ represents an active error.
    double speed_ratio_;                  ///< Global speed override (0.0 to 1.0).
    bool estop_active_;                   ///< Emergency stop status.
    bool is_physically_connected_;        ///< Status of physical connection to hardware.

    // --- Mutexes for thread-safe access ---
    // Using mutable to allow locking in const getter methods.
    mutable std::shared_mutex mutex_cmd_pose_;
    mutable std::shared_mutex mutex_fb_pose_;
    mutable std::shared_mutex mutex_active_tool_;
    mutable std::shared_mutex mutex_active_base_frame_;
    mutable std::shared_mutex mutex_robot_mode_;
    mutable std::shared_mutex mutex_actual_joint_state_;
    mutable std::shared_mutex mutex_system_message_; // Covers message and has_error_
    mutable std::shared_mutex mutex_speed_ratio_;
    mutable std::shared_mutex mutex_estop_state_;
    mutable std::shared_mutex mutex_connection_status_;

    static inline const std::string MODULE_NAME = "StateData"; // For logging (if used internally)
};

} // namespace RDT
#endif // STATEDATA_H