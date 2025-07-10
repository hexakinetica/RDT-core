// RobotController.cpp
#include "RobotController.h"
#include <stdexcept> // For std::invalid_argument, std::runtime_error
#include <vector>    // For std::vector
#include <chrono>    // For std::chrono literals and types
#include <thread>    // For std::this_thread::sleep_for
#include <cmath>     // For std::ceil
#include <functional> // For std::placeholders

namespace RDT {

using namespace RDT::literals;
using namespace std::chrono_literals;

RobotController::RobotController(std::shared_ptr<TrajectoryPlanner> planner,
                                 std::shared_ptr<MotionManager> motion_manager,
                                 std::shared_ptr<KinematicSolver> solver,
                                 std::shared_ptr<StateData> state_data)
    : planner_(std::move(planner)),
      motion_manager_(std::move(motion_manager)),
      solver_(std::move(solver)),
      state_data_(std::move(state_data)) {
    if (!planner_) {
        LOG_CRITICAL(MODULE_NAME, "TrajectoryPlanner dependency is null during construction.");
        throw std::invalid_argument("RobotController: TrajectoryPlanner cannot be null.");
    }
    if (!motion_manager_) {
        LOG_CRITICAL(MODULE_NAME, "MotionManager dependency is null during construction.");
        throw std::invalid_argument("RobotController: MotionManager cannot be null.");
    }
    if (!solver_) {
        LOG_CRITICAL(MODULE_NAME, "KinematicSolver dependency is null during construction.");
        throw std::invalid_argument("RobotController: KinematicSolver cannot be null.");
    }
    if (!state_data_) {
        LOG_CRITICAL(MODULE_NAME, "StateData dependency is null during construction.");
        throw std::invalid_argument("RobotController: StateData cannot be null.");
    }
    logControllerMessage("RobotController instance created.", LogLevel::Info);
}

RobotController::~RobotController() {
    logControllerMessage("RobotController shutting down...", LogLevel::Info);
    stopControlLoop(); //    
    logControllerMessage("RobotController shutdown complete.", LogLevel::Info);
}

void RobotController::logControllerMessage(const std::string& message, LogLevel level) {
    Logger::log(level, MODULE_NAME, message);
    if (level >= LogLevel::Error && state_data_) {
        state_data_->setSystemMessage(message, true);
    }
}

void RobotController::startControlLoop() {
 stopControlLoop(); // ,    
    //  -    stop_token
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

bool RobotController::initialize(const TrajectoryPoint& initial_robot_state_user) {
    logControllerMessage("Initializing RobotController...", LogLevel::Info);
    
    stopControlLoop();

    //state_data_->reset();
    state_data_->setRobotMode(RobotMode::Initializing);

    active_user_tool_context_ = initial_robot_state_user.header.tool;
    active_user_base_context_ = initial_robot_state_user.header.base;
    state_data_->setActiveToolFrame(active_user_tool_context_);
    state_data_->setActiveBaseFrame(active_user_base_context_);

    TrajectoryPoint initial_state_flange_base;
    initial_state_flange_base.header.tool = ToolFrame();
    initial_state_flange_base.header.base = BaseFrame();

    if (initial_robot_state_user.command.joint_target.size() == ROBOT_AXES_COUNT) {
        initial_state_flange_base.command.joint_target = initial_robot_state_user.command.joint_target;
    } else {
        logControllerMessage("Initial state must provide valid joint positions.", LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    try {
        initial_state_flange_base.command.cartesian_target = solver_->solveFK(initial_state_flange_base.command.joint_target);
    } catch (const std::exception& e) {
        logControllerMessage("FK failed for initial joint state: " + std::string(e.what()), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    state_data_->setCmdTrajectoryPoint(initial_robot_state_user);
    TrajectoryPoint initial_fb_for_sdata;
    initial_fb_for_sdata.header = initial_robot_state_user.header;
    initial_fb_for_sdata.feedback.joint_actual = initial_state_flange_base.command.joint_target;
    initial_fb_for_sdata.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
        initial_state_flange_base.command.cartesian_target,
        active_user_tool_context_.transform
    );
    initial_fb_for_sdata.feedback.rt_state = RTState::Idle;
    state_data_->setFbTrajectoryPoint(initial_fb_for_sdata);
    
    planner_->setCurrentRobotState(initial_state_flange_base);
    if (planner_->hasError()) {
        logControllerMessage("Planner initialization error: " + planner_->getErrorMessage(), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    motion_manager_->reset();
    if (!motion_manager_->start()) {
        logControllerMessage("MotionManager failed to start.", LogLevel::Critical);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    startControlLoop();
    state_data_->setRobotMode(RobotMode::Idle);
    state_data_->setSystemMessage("System Initialized. Ready.", false);
    logControllerMessage("RobotController initialized and Idle.", LogLevel::Info);
    return true;
}

bool RobotController::executeMotionToTarget(const TrajectoryPoint& target_waypoint_user_frame) {
    // Fix: Convert radians to degrees manually

    std::string A3_temp = target_waypoint_user_frame.command.joint_target.toJointPoseString();
    logControllerMessage(A3_temp + " Attempting to execute motion to new target.", LogLevel::Info);

    const RobotMode current_mode = state_data_->getRobotMode();
    if (current_mode != RobotMode::Idle) {
        logControllerMessage("Cannot execute motion: Controller not in Idle state. Current mode: " + std::to_string(static_cast<int>(current_mode)), LogLevel::Warning);
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

    active_user_tool_context_ = target_waypoint_user_frame.header.tool;
    active_user_base_context_ = target_waypoint_user_frame.header.base;
    current_user_command_context_ = target_waypoint_user_frame;
    state_data_->setActiveToolFrame(active_user_tool_context_);
    state_data_->setActiveBaseFrame(active_user_base_context_);
    state_data_->setCmdTrajectoryPoint(target_waypoint_user_frame);

    TrajectoryPoint current_sdata_fb = state_data_->getFbTrajectoryPoint();
    TrajectoryPoint planner_start_state_flange_base;
    planner_start_state_flange_base.command.joint_target = current_sdata_fb.feedback.joint_actual;
    planner_start_state_flange_base.command.cartesian_target = FrameTransformer::calculateFlangeInWorld(
        current_sdata_fb.feedback.cartesian_actual,
        current_sdata_fb.header.tool.transform
    );
    planner_start_state_flange_base.header.tool = ToolFrame();
    planner_start_state_flange_base.header.base = BaseFrame();

    planner_->setCurrentRobotState(planner_start_state_flange_base);
    if (planner_->hasError()) {
        logControllerMessage("Planner error on set current state: " + planner_->getErrorMessage(), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    if (!planner_->addTargetWaypoint(target_waypoint_user_frame)) {
        std::string err_msg = "Failed to add target to planner: " + planner_->getErrorMessage();
        logControllerMessage(err_msg, LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return false;
    }

    state_data_->setRobotMode(RobotMode::Running);
    logControllerMessage("Motion task initiated.", LogLevel::Info);
    return true;
}

void RobotController::controlLoop(std::stop_token stoken) {
    last_feedback_processing_time_ = std::chrono::steady_clock::now();

    while (!stoken.stop_requested()) {
        auto loop_start_time = std::chrono::steady_clock::now();
        
        if ((loop_start_time - last_feedback_processing_time_) >= FEEDBACK_PROCESSING_INTERVAL) {
            processFeedbackBatch();
            last_feedback_processing_time_ = loop_start_time;
        }

        const RobotMode current_mode = state_data_->getRobotMode();

        switch (current_mode) {
            case RobotMode::Running:
                processMotionTask();
                break;
            case RobotMode::Idle:
            case RobotMode::Initializing:
            case RobotMode::EStop:
            case RobotMode::Error:
                // No action needed in these states
                break;
        }

        auto loop_end_time = std::chrono::steady_clock::now();
        auto duration = loop_end_time - loop_start_time;
        if (duration < CONTROL_LOOP_PERIOD) {
            std::this_thread::sleep_for(CONTROL_LOOP_PERIOD - duration);
        } else {
             LOG_WARN_F(MODULE_NAME, "Control loop overrun! Duration: %lld ms",
                       std::chrono::duration_cast<std::chrono::milliseconds>(duration).count());
        }
    }
}

void RobotController::processFeedbackBatch() {
    std::vector<TrajectoryPoint> feedback_batch;
    while (auto fb_opt = motion_manager_->dequeueFeedback()) {
        feedback_batch.push_back(*fb_opt);
    }
    if (feedback_batch.empty()) {
        return;
    }

    TrajectoryPoint& latest_feedback = feedback_batch.back();
    TrajectoryPoint fb_for_sdata;
    
    // Header  Command      
    fb_for_sdata.header = current_user_command_context_.header;
    fb_for_sdata.command = current_user_command_context_.command;
    fb_for_sdata.feedback.joint_actual = latest_feedback.feedback.joint_actual;
    fb_for_sdata.feedback.rt_state = latest_feedback.feedback.rt_state;

    try {
        // FK        
        CartPose flange_pose = solver_->solveFK(latest_feedback.feedback.joint_actual);
        //     TCP    
        fb_for_sdata.feedback.cartesian_actual = FrameTransformer::calculateTcpInWorld(
            flange_pose,
            active_user_tool_context_.transform
        );
    } catch (const std::exception& e) {
        logControllerMessage("FK failed for feedback processing: " + std::string(e.what()), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return;
    }

    LOG_DEBUG_F(MODULE_NAME, "Updating StateData with latest feedback from a batch of %zu points.", feedback_batch.size());
    state_data_->setFbTrajectoryPoint(fb_for_sdata);

    //  RobotMode,         /EStop
    RobotMode current_mode = state_data_->getRobotMode();
    if (current_mode != RobotMode::Error && current_mode != RobotMode::EStop) {
        //state_data_->setRobotMode(static_cast<RobotMode>(latest_feedback.feedback.rt_state));
    }
}

void RobotController::processMotionTask() {
    if (planner_->hasError()) {
        logControllerMessage("Task failed (planner error): " + planner_->getErrorMessage(), LogLevel::Error);
        state_data_->setRobotMode(RobotMode::Error);
        return;
    }

    if (planner_->isCurrentSegmentDone()) {
        logControllerMessage("Motion task completed. Controller to Idle.", LogLevel::Info);
        state_data_->setRobotMode(RobotMode::Idle);
        return;
    }

    const auto max_points_per_window = static_cast<std::size_t>(std::ceil(PLANNER_WINDOW_DURATION.value() / PLANNER_DT_SAMPLE.value()));
    const std::size_t desired_mm_buffer_level = max_points_per_window * 1; 

    if (motion_manager_->getCommandQueueSize() < desired_mm_buffer_level) {
        std::vector<TrajectoryPoint> window = planner_->getNextPointWindow(PLANNER_DT_SAMPLE, PLANNER_WINDOW_DURATION);

        if (planner_->hasError()) {
            logControllerMessage("Task failed (planner window gen error): " + planner_->getErrorMessage(), LogLevel::Error);
            state_data_->setRobotMode(RobotMode::Error);
            return;
        }

        if (!window.empty()) {
            for (const TrajectoryPoint& tp : window) {
                if (!motion_manager_->enqueueCommand(tp)) {
                    logControllerMessage("Failed to enqueue point to MotionManager (queue full).", LogLevel::Warning);
                    break;
                }
            }
        }
    }
}

void RobotController::cancelCurrentMotion() {
    logControllerMessage("Cancel current motion requested.", LogLevel::Info);
    
    state_data_->setRobotMode(RobotMode::Idle);
    state_data_->setSystemMessage("Motion task cancelled by user.", false);

    if (motion_manager_) {
        motion_manager_->reset();
    }
    
    //       
    TrajectoryPoint last_known_fb_sdata = state_data_->getFbTrajectoryPoint();
    TrajectoryPoint planner_stop_state_flange_base;
    planner_stop_state_flange_base.command.joint_target = last_known_fb_sdata.feedback.joint_actual;
    try {
        planner_stop_state_flange_base.command.cartesian_target = solver_->solveFK(planner_stop_state_flange_base.command.joint_target);
    } catch(const std::exception& e) {
        logControllerMessage("Error on FK during cancel: " + std::string(e.what()), LogLevel::Error);
    }
    planner_stop_state_flange_base.header.tool = ToolFrame();
    planner_stop_state_flange_base.header.base = BaseFrame();
    planner_->setCurrentRobotState(planner_stop_state_flange_base);
    if(planner_->hasError()){
        planner_->clearError();
    }
    logControllerMessage("Motion cancelled. Planner reset to last known position.", LogLevel::Info);
}

void RobotController::emergencyStop() {
    logControllerMessage("EMERGENCY STOP ACTIVATED!", LogLevel::Critical);
    
    state_data_->setEstopState(true);
    state_data_->setRobotMode(RobotMode::EStop);
    state_data_->setSystemMessage("EMERGENCY STOP ACTIVATED", true);
    
    if (motion_manager_) {
        motion_manager_->emergencyStop(); 
    }
    if (planner_ && planner_->hasError()){
        planner_->clearError();
    }
}

void RobotController::reset() {
    logControllerMessage("Full system reset initiated.", LogLevel::Info);
    
    stopControlLoop();

    if (motion_manager_) {
        motion_manager_->stop();
        motion_manager_->reset();
    }
    if (planner_) {
        planner_->clearError();
    }
    
    //state_data_->reset();
    state_data_->setRobotMode(RobotMode::Initializing);
    state_data_->setSystemMessage("System reset. Awaiting re-initialization.", false);

    logControllerMessage("System reset complete. Call initialize() to restart.", LogLevel::Info);
}

ControllerState RobotController::getInternalControllerState() const {
    RobotMode mode = state_data_->getRobotMode();
    switch (mode) {
        case RobotMode::Idle:         return ControllerState::Idle;
        case RobotMode::Running:       return ControllerState::Running;
        case RobotMode::Initializing: return ControllerState::Initializing;
        case RobotMode::EStop:        return ControllerState::Error;
        case RobotMode::Error:        return ControllerState::Error;
        default:                      return ControllerState::Idle;
    }
}

bool RobotController::isMotionTaskActive() const {
    return state_data_->getRobotMode() == RobotMode::Running;
}

} // namespace RDT