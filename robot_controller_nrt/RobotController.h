// RobotController.h
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "Logger.h"
#include "StateData.h"
#include "MasterHardwareInterface.h"
#include "MotionManager.h"
#include "TrajectoryPlanner.h"
#include "TrajectoryInterpolator.h"
#include "KinematicSolver.h"
#include "KinematicModel.h"
#include "KdlKinematicSolver.h"
#include "RobotConfig.h" // *** NEW DEPENDENCY ***


#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>
#include <vector>

namespace RDT {

enum class ControllerState { Idle, Initializing, Running, Error };

class RobotController {
public:
    RobotController(const InterfaceConfig& hw_config,
                    const ControllerConfig& ctrl_config, // *** NEW ***
                    const KinematicModel& model,
                    std::shared_ptr<StateData> state_data);
    ~RobotController();

    RobotController(const RobotController&) = delete;
    RobotController& operator=(const RobotController&) = delete;
    RobotController(RobotController&&) = delete;
    RobotController& operator=(RobotController&&) = delete;

    bool initialize(const TrajectoryPoint& initial_robot_state_user);
    [[nodiscard]] bool executeMotionToTarget(const TrajectoryPoint& target_waypoint_user_frame);
    void cancelCurrentMotion();
    void emergencyStop();
    void reset();

    enum class SwitchRequestResult { Success, Error, NotInSync, AlreadyInMode };
    [[nodiscard]] SwitchRequestResult requestModeSwitch(MasterHardwareInterface::ActiveMode mode);
    void forceSync();

    [[nodiscard]] ControllerState getInternalControllerState() const;
    [[nodiscard]] bool isMotionTaskActive() const;

    // ============================================================================
    // *** НОВЫЕ МЕТОДЫ-ПРОКСИ для Adapter ***
    // ============================================================================
    [[nodiscard]] MasterHardwareInterface::ActiveMode getActiveMode() const;
    [[nodiscard]] bool isRealInterfaceConnected() const;

private:
    void controlLoop(std::stop_token stoken);
    void processMotionTask();
    void processFeedbackBatch();

    void processFault(const FaultData& fault);

    void logControllerMessage(const std::string& message, LogLevel level = LogLevel::Info);
    void startControlLoop();
    void stopControlLoop();

    std::shared_ptr<StateData> state_data_;
    std::shared_ptr<KinematicSolver> solver_;
    std::shared_ptr<TrajectoryPlanner> planner_;
    std::shared_ptr<MotionManager> motion_manager_;
    std::shared_ptr<MasterHardwareInterface> hw_interface_;

    std::jthread control_thread_;
    std::atomic<bool> motion_task_active_{false};
    TrajectoryPoint current_user_command_context_;
    std::chrono::steady_clock::time_point last_feedback_processing_time_;

    // *** MODIFIED: Parameters from config instead of static consts ***
    const ControllerConfig config_;

    static inline const std::string MODULE_NAME = "RobotCtrl";
};

} // namespace RDT
#endif // ROBOTCONTROLLER_H