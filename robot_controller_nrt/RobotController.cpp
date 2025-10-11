// RobotController.cpp
#include "RobotController.h"
#include <stdexcept>

namespace RDT {

using namespace RDT::literals;
using namespace std::chrono_literals;

RobotController::RobotController(const InterfaceConfig& hw_config,
                                 const ControllerConfig& ctrl_config,
                                 const KinematicModel& model,
                                 std::shared_ptr<StateData> state_data)
    : state_data_(state_data),
      config_(ctrl_config) {

    if (!state_data_) {
        throw std::invalid_argument("RobotController: StateData cannot be null.");
    }
    
    logControllerMessage("Constructing RobotController and its components...", LogLevel::Info);
    
    hw_interface_ = std::make_shared<MasterHardwareInterface>(hw_config);
    
    motion_manager_ = std::make_shared<MotionManager>(
        hw_interface_,
        config_.motion_manager_cycle_ms,
        model.getLimits()
    );
    
    solver_ = std::make_shared<KdlKinematicSolver>(model);
    
    auto interpolator = std::make_shared<TrajectoryInterpolator>();
    planner_ = std::make_shared<TrajectoryPlanner>(solver_, interpolator);

    logControllerMessage("RobotController instance created.", LogLevel::Info);
}

RobotController::~RobotController() {
    logControllerMessage("RobotController shutting down...", LogLevel::Info);
    stopControlLoop();
    if (motion_manager_) motion_manager_->stop();
    if (hw_interface_) hw_interface_->disconnect();
    logControllerMessage("RobotController shutdown complete.", LogLevel::Info);
}

bool RobotController::initialize(const TrajectoryPoint& initial_robot_state_user) {
    logControllerMessage("Initializing RobotController...", LogLevel::Info);
    
    stopControlLoop();
    if (motion_manager_) motion_manager_->stop();
    if (hw_interface_) hw_interface_->disconnect();

    state_data_->setRobotMode(RobotMode::Initializing);
    
    if (!hw_interface_->connect()) {
        logControllerMessage("Hardware interface failed to connect.", LogLevel::Critical);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }
    
    hw_interface_->setState(initial_robot_state_user.command.joint_target);

    if (!motion_manager_->start()) {
        logControllerMessage("MotionManager failed to start.", LogLevel::Critical);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }
    
    AxisSet initial_joints = hw_interface_->getLatestFeedback();
    CartPose initial_flange_pose;
    if (!solver_->solveFK(initial_joints, initial_flange_pose)) {
        logControllerMessage("FK failed for initial joint state.", LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    state_data_->setActiveToolFrame(initial_robot_state_user.header.tool);
    state_data_->setActiveBaseFrame(initial_robot_state_user.header.base);

    TrajectoryPoint initial_fb_for_sdata;
    initial_fb_for_sdata.header = initial_robot_state_user.header;
    initial_fb_for_sdata.feedback.joint_actual = initial_joints;
    initial_fb_for_sdata.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
        initial_flange_pose, initial_robot_state_user.header.tool.transform
    );
    state_data_->setFbTrajectoryPoint(initial_fb_for_sdata);
    
    startControlLoop();
    state_data_->setRobotMode(RobotMode::Idle);
    state_data_->setSystemMessage("System Initialized. Ready.", false);
    logControllerMessage("RobotController initialized and Idle.", LogLevel::Info);
    return true;
}

[[nodiscard]] bool RobotController::executeMotionToTarget(const TrajectoryPoint& target_waypoint_user_frame) {
    if (motion_task_active_.load()) {
        logControllerMessage("Cannot execute motion: another motion task is already active.", LogLevel::Warning);
        return false;
    }
    if (state_data_->isEstopActive()) {
        logControllerMessage("Cannot execute motion: E-Stop is active.", LogLevel::Error);
        return false;
    }

    if (state_data_->hasActiveError()) {
        state_data_->setSystemMessage("Previous error acknowledged. Attempting new motion.", false);
    }
    if (planner_->hasError()) {
        planner_->clearError();
    }

    current_user_command_context_ = target_waypoint_user_frame;
    state_data_->setActiveToolFrame(target_waypoint_user_frame.header.tool);
    state_data_->setActiveBaseFrame(target_waypoint_user_frame.header.base);
    state_data_->setCmdTrajectoryPoint(target_waypoint_user_frame);

    TrajectoryPoint current_sdata_fb = state_data_->getFbTrajectoryPoint();
    TrajectoryPoint planner_start_state_flange_base;
    planner_start_state_flange_base.command.joint_target = current_sdata_fb.feedback.joint_actual;
    if (!solver_->solveFK(planner_start_state_flange_base.command.joint_target, planner_start_state_flange_base.command.cartesian_target)) {
        logControllerMessage("Failed to calculate start pose for planner.", LogLevel::Error);
        return false;
    }
    
    planner_->setCurrentRobotState(planner_start_state_flange_base);
    if (!planner_->addTargetWaypoint(target_waypoint_user_frame)) {
        logControllerMessage("Failed to add target to planner: " + planner_->getErrorMessage(), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    motion_task_active_.store(true);
    state_data_->setRobotMode(RobotMode::Running);
    logControllerMessage("Motion task initiated.", LogLevel::Info);
    return true;
}

void RobotController::controlLoop(std::stop_token stoken) {
    last_feedback_processing_time_ = std::chrono::steady_clock::now();
    while (!stoken.stop_requested()) {
        auto loop_start_time = std::chrono::steady_clock::now();
        
        // *** ИСПРАВЛЕНИЕ: Используем поле config_ ***
        if ((loop_start_time - last_feedback_processing_time_) >= config_.feedback_processing_interval) {
            processFeedbackBatch();
            last_feedback_processing_time_ = loop_start_time;
        }
        
        if (motion_task_active_.load()) {
            processMotionTask();
        }

        auto loop_end_time = std::chrono::steady_clock::now();
        auto duration = loop_end_time - loop_start_time;
        // *** ИСПРАВЛЕНИЕ: Используем поле config_ ***
        if (duration < config_.control_loop_period) {
            std::this_thread::sleep_for(config_.control_loop_period - duration);
        }
    }
}

// *** MODIFIED: Упрощенный обработчик батча обратной связи ***
void RobotController::processFeedbackBatch() {
    TrajectoryPoint latest_feedback;
    bool has_new_feedback = false;
    while (motion_manager_->dequeueFeedback(latest_feedback)) {
        has_new_feedback = true;
    }

    if (!has_new_feedback) {
        return; // No new data to process
    }

    // --- ШАГ 1: Делегируем обработку ошибок ---
    if (latest_feedback.feedback.fault.type != FaultType::None) {
        processFault(latest_feedback.feedback.fault);
    }

    // --- ШАГ 2: Обновляем StateData (как и раньше) ---
    TrajectoryPoint fb_for_sdata;
    fb_for_sdata.header = current_user_command_context_.header;
    fb_for_sdata.feedback = latest_feedback.feedback;

    CartPose flange_pose;
    if (solver_->solveFK(fb_for_sdata.feedback.joint_actual, flange_pose)) {
        fb_for_sdata.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
            flange_pose, state_data_->getActiveToolFrame().transform);
    } else {
        logControllerMessage("FK failed during feedback processing.", LogLevel::Warning);
    }
    state_data_->setFbTrajectoryPoint(fb_for_sdata);

    // Обновляем RobotMode только если не в критическом состоянии
    if (state_data_->getRobotMode() != RobotMode::Error && state_data_->getRobotMode() != RobotMode::EStop) {
        // Преобразуем RTState (0,2,4..) в RobotMode (0,1,3..)
        switch(latest_feedback.feedback.rt_state) {
            case RTState::Idle:
                // Если планировщик тоже закончил, то мы реально Idle
                if (planner_->isCurrentSegmentDone()) {
                    state_data_->setRobotMode(RobotMode::Idle);
                }
                break;
            case RTState::Moving:
                state_data_->setRobotMode(RobotMode::Running);
                break;
            case RTState::Error:
                state_data_->setRobotMode(RobotMode::Error);
                break;
            default:
                break;
        }
    }
}


// *** NEW: Выделенный обработчик ошибок ***
void RobotController::processFault(const FaultData& fault) {
    std::stringstream ss;

    switch (fault.type) {
        case FaultType::PositionLimit:
            ss << "CRITICAL ERROR: Position Limit on Axis " << (static_cast<int>(fault.axis) + 1)
               << ". Commanded: " << fault.actual_position_deg.toString(2)
               << ", Limit: " << fault.limit_position_deg.toString(2);
            
            logControllerMessage(ss.str(), LogLevel::Error);
            motion_task_active_.store(false); // Немедленно остановить текущую задачу
            state_data_->setRobotMode(RobotMode::Error); // Перевести систему в состояние ошибки
            break;

        case FaultType::VelocityLimit:
            ss << "WARNING: Velocity Limit on Axis " << (static_cast<int>(fault.axis) + 1)
               << ". Required: " << fault.actual_velocity_deg_s.toString(1)
               << ", Limit: " << fault.limit_velocity_deg_s.toString(1);
            
            logControllerMessage(ss.str(), LogLevel::Warning);
            // Ничего не делаем, движение не останавливаем. MotionManager уже справился.
            break;

        case FaultType::None:
            // Ничего не делаем
            break;
    }
}


void RobotController::processMotionTask() {
    if (planner_->hasError()) {
        logControllerMessage("Task failed (planner error): " + planner_->getErrorMessage(), LogLevel::Error);
        motion_task_active_.store(false);
        state_data_->setRobotMode(RobotMode::Error);
        return;
    }

    if (planner_->isCurrentSegmentDone()) {
        logControllerMessage("Motion task completed. Controller to Idle.", LogLevel::Info);
        motion_task_active_.store(false);
        state_data_->setRobotMode(RobotMode::Idle);
        return;
    }

    if (motion_manager_->getCommandQueueSize() < 10) {
        // *** ИСПРАВЛЕНИЕ: Используем поле config_ ***
        std::vector<TrajectoryPoint> window = planner_->getNextPointWindow(config_.planner_dt_sample, config_.planner_window_duration);
        if (planner_->hasError()) {
            logControllerMessage("Task failed (planner window gen error): " + planner_->getErrorMessage(), LogLevel::Error);
            motion_task_active_.store(false);
            state_data_->setRobotMode(RobotMode::Error);
            return;
        }
        for (const auto& tp : window) {
            if (!motion_manager_->enqueueCommand(tp)) {
                logControllerMessage("Failed to enqueue point to MotionManager (queue full).", LogLevel::Warning);
                break;
            }
        }
    }
}

void RobotController::cancelCurrentMotion() {
    logControllerMessage("Cancel current motion requested.", LogLevel::Info);
    motion_task_active_.store(false);
    state_data_->setRobotMode(RobotMode::Idle);
    if (motion_manager_) motion_manager_->reset();
    if (planner_) {
        planner_->clearError();
        TrajectoryPoint current_fb = state_data_->getFbTrajectoryPoint();
        current_fb.command.joint_target = current_fb.feedback.joint_actual;
        planner_->setCurrentRobotState(current_fb);
    }
}

void RobotController::emergencyStop() {
    logControllerMessage("EMERGENCY STOP ACTIVATED!", LogLevel::Critical);
    motion_task_active_.store(false);
    state_data_->setEstopState(true);
    state_data_->setRobotMode(RobotMode::EStop);
    if (motion_manager_) motion_manager_->emergencyStop();
}

void RobotController::reset() {
    logControllerMessage("Soft reset initiated.", LogLevel::Info);
    cancelCurrentMotion();
    state_data_->setEstopState(false);
    state_data_->setSystemMessage("System Reset.", false);
    state_data_->setRobotMode(RobotMode::Idle);
}

RobotController::SwitchRequestResult RobotController::requestModeSwitch(MasterHardwareInterface::ActiveMode mode) {
    if (isMotionTaskActive()) {
        logControllerMessage("Cannot switch mode while motion task is active.", LogLevel::Error);
        return SwitchRequestResult::Error;
    }
    if (hw_interface_->getCurrentMode() == mode) {
        return SwitchRequestResult::AlreadyInMode;
    }

    auto result = hw_interface_->switchTo(mode);

    if (result == MasterHardwareInterface::SwitchResult::NotInSync) {
        logControllerMessage("Mode switch rejected: robot is not synchronized.", LogLevel::Warning);
        return SwitchRequestResult::NotInSync;
    }
    if (result == MasterHardwareInterface::SwitchResult::Success) {
        AxisSet new_feedback_joints = hw_interface_->getLatestFeedback();
        TrajectoryPoint current_fb = state_data_->getFbTrajectoryPoint();
        current_fb.feedback.joint_actual = new_feedback_joints;
        
        CartPose flange_pose;
        if (solver_->solveFK(new_feedback_joints, flange_pose)) {
            current_fb.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
                flange_pose, state_data_->getActiveToolFrame().transform);
        }
        state_data_->setFbTrajectoryPoint(current_fb);
        return SwitchRequestResult::Success;
    }
    return SwitchRequestResult::Error;
}

void RobotController::forceSync() {
    if (isMotionTaskActive()) {
        logControllerMessage("Cannot sync while motion task is active.", LogLevel::Warning);
        return;
    }
    hw_interface_->forceSyncSimulationToReal();
    AxisSet synced_joints = hw_interface_->getLatestFeedback();
    TrajectoryPoint current_fb = state_data_->getFbTrajectoryPoint();
    current_fb.feedback.joint_actual = synced_joints;
    
    CartPose flange_pose;
    if (solver_->solveFK(synced_joints, flange_pose)) {
        current_fb.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
            flange_pose, state_data_->getActiveToolFrame().transform);
    }
    state_data_->setFbTrajectoryPoint(current_fb);
    logControllerMessage("Simulation state synchronized with real hardware.", LogLevel::Info);
}


ControllerState RobotController::getInternalControllerState() const {
    if (state_data_->isEstopActive() || state_data_->hasActiveError()) {
        return ControllerState::Error;
    }
    if (motion_task_active_.load()) {
        return ControllerState::Running;
    }
    if (state_data_->getRobotMode() == RobotMode::Initializing) {
        return ControllerState::Initializing;
    }
    return ControllerState::Idle;
}

bool RobotController::isMotionTaskActive() const {
    return motion_task_active_.load();
}

void RobotController::startControlLoop() {
    stopControlLoop();
    control_thread_ = std::jthread([this](std::stop_token stoken) {
        this->controlLoop(stoken);
    });
    logControllerMessage("Control loop started.", LogLevel::Info);
}

void RobotController::stopControlLoop() {
    if (control_thread_.joinable()) {
        control_thread_.request_stop();
        control_thread_.join();
        logControllerMessage("Control loop thread joined.", LogLevel::Info);
    }
}

void RobotController::logControllerMessage(const std::string& message, LogLevel level) {
    // *** ИСПРАВЛЕНИЕ: Макрос логгера требует 3 аргумента ***
    Logger::log(level, MODULE_NAME, message);
    if (level >= LogLevel::Error && state_data_) {
        state_data_->setSystemMessage(message, true);
    }
}


// *** НОВАЯ РЕАЛИЗАЦИЯ: Методы-прокси ***
MasterHardwareInterface::ActiveMode RobotController::getActiveMode() const {
    if (hw_interface_) {
        return hw_interface_->getCurrentMode();
    }
    return MasterHardwareInterface::ActiveMode::Simulation; // Безопасное значение по умолчанию
}

bool RobotController::isRealInterfaceConnected() const {
    if (hw_interface_) {
        // Этот метод нужно добавить в MasterHardwareInterface
        return hw_interface_->isRealInterfaceConnected();
    }
    return false;
}


} // namespace RDT