// RobotController.h
#ifndef ROBOTCONTROLLER_H
#define ROBOTCONTROLLER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "TrajectoryPlanner.h"
#include "MotionManager.h"
#include "KinematicSolver.h"
#include "StateData.h"
#include "FrameTransformer.h"
#include "Logger.h"

#include <memory>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <chrono>

namespace RDT {

// ControllerState      ,
//   StateData->RobotMode  .
//       ,  -  .
enum class ControllerState { Idle, Initializing, Running, Moving, Paused, Error, Stopping };

class RobotController {
public:
    RobotController(std::shared_ptr<TrajectoryPlanner> planner,
                    std::shared_ptr<MotionManager> motion_manager,
                    std::shared_ptr<KinematicSolver> solver,
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

    //    .       StateData.
    [[nodiscard]] ControllerState getInternalControllerState() const;
    [[nodiscard]] bool isMotionTaskActive() const;

private:
    void controlLoop(std::stop_token stoken); //  stop_token  jthread
    void processMotionTask();
    void processFeedbackBatch(); //      

    void logControllerMessage(const std::string& message, LogLevel level = LogLevel::Info);
    void startControlLoop(); // Helper   
    void stopControlLoop();  // Helper   

    std::shared_ptr<TrajectoryPlanner> planner_;
    std::shared_ptr<MotionManager> motion_manager_;
    std::shared_ptr<KinematicSolver> solver_;
    std::shared_ptr<StateData> state_data_;

    std::jthread control_thread_;
    //    . StateData -   .
    // std::atomic<ControllerState> internal_controller_state_ {ControllerState::Idle};
    // std::atomic<bool> current_motion_task_active_ {false};
    // std::atomic<bool> estop_active_ {false};

    ToolFrame active_user_tool_context_;
    BaseFrame active_user_base_context_;
    TrajectoryPoint current_user_command_context_;
    
    //    
    std::chrono::steady_clock::time_point last_feedback_processing_time_;

    static constexpr std::chrono::milliseconds CONTROL_LOOP_PERIOD{std::chrono::milliseconds(50)}; //      
    static constexpr std::chrono::milliseconds FEEDBACK_PROCESSING_INTERVAL{std::chrono::milliseconds(200)}; //   
    static constexpr Seconds PLANNER_DT_SAMPLE = 0.1_s;
    static constexpr Seconds PLANNER_WINDOW_DURATION = 1.0_s;

    static inline const std::string MODULE_NAME = "RobotCtrl";
};

} // namespace RDT
#endif // ROBOTCONTROLLER_H