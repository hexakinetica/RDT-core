// MotionManager.cpp
#include "MotionManager.h"
#include <stdexcept> // For std::invalid_argument
#include <chrono>    // For std::chrono literals
#include <iostream>  // Temporary, replace with logger if too verbose in RT
#include <sstream>   // For std::ostringstream



namespace RDT {

std::string Axis2Str(AxisSet axis){
    // Fixed typo and formatting
    std::ostringstream oss;
    oss << "Joint, DEG: [A1: " << axis[AxisId::A1].angle.toDegrees().value()
        << ", A2: " << axis[AxisId::A2].angle.toDegrees().value()
        << ", A3: " << axis[AxisId::A3].angle.toDegrees().value()
        << ", A4: " << axis[AxisId::A4].angle.toDegrees().value()
        << ", A5: " << axis[AxisId::A5].angle.toDegrees().value()
        << ", A6=" << axis[AxisId::A6].angle.toDegrees().value()
        << "]";
    return oss.str();
}

using namespace std::chrono_literals; // For 10ms, etc.

MotionManager::MotionManager(std::unique_ptr<IMotionInterface> motion_interface,
                             unsigned int cycle_period_ms)
    : motion_interface_(std::move(motion_interface)),
      cycle_period_(cycle_period_ms),
      current_status_message_("Initialized") {
    if (!motion_interface_) {
        LOG_CRITICAL(MODULE_NAME, "IMotionInterface cannot be null.");
        throw std::invalid_argument("MotionManager: IMotionInterface cannot be null.");
    }
    if (cycle_period_ms == 0) {
        LOG_CRITICAL(MODULE_NAME, "Invalid cycle period: must be > 0 ms.");
        throw std::invalid_argument("MotionManager: Invalid cycle period.");
    }
    LOG_INFO_F(MODULE_NAME, "MotionManager created with cycle period: %u ms.", cycle_period_ms);
    // Initialize current_rt_command_point_ to a safe default (e.g., zero positions)
    // This will be used if the command queue is empty to "hold position".
    // Assuming RobotCommandFrame and AxisSet default constructors create zero/idle states.
    current_rt_command_point_.command = RobotCommandFrame{}; 
    current_rt_command_point_.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD; // Or appropriate default
}

MotionManager::~MotionManager() {
    LOG_INFO(MODULE_NAME, "MotionManager shutting down...");
    stop(); // Ensure thread is stopped and joined
    LOG_INFO(MODULE_NAME, "MotionManager shut down complete.");
}

bool MotionManager::start() {
    if (running_.load()) {
        LOG_WARN(MODULE_NAME, "RT Loop already running.");
        return false;
    }
    if (!motion_interface_->isConnected()) {
        LOG_INFO(MODULE_NAME, "Motion interface not connected. Attempting to connect...");
        if (!motion_interface_->connect()) {
            LOG_ERROR(MODULE_NAME, "Failed to connect motion interface. Cannot start RT loop.");
            // Set an error state
            {
                std::lock_guard<std::mutex> lock(status_mutex_);
                current_status_message_ = "Failed to connect to motion interface.";
                current_error_code_ = -1; // Example error code
            }
            current_mode_.store(RTState::Error);
            return false;
        }
        //return true;
    }

    LOG_INFO(MODULE_NAME, "Starting RT Loop...");
    running_ = true;
    estop_active_ = false; // Reset E-Stop flag on start
    current_mode_.store(RTState::Idle); // Start in Idle
    rt_thread_ = std::jthread(&MotionManager::rt_cycle_tick, this);
    LOG_INFO(MODULE_NAME, "RT Loop started.");
    return true;
}

void MotionManager::stop() {
    LOG_INFO(MODULE_NAME, "Stopping RT Loop...");
    running_ = false; // Signal thread to stop its main work
    if (rt_thread_.joinable()) {
        rt_thread_.request_stop(); // Request stop for the jthread
        if (rt_thread_.joinable()) { // Check again before join, request_stop might be fast
             rt_thread_.join();
        }
        LOG_INFO(MODULE_NAME, "RT Loop stopped and thread joined.");
    } else {
        LOG_INFO(MODULE_NAME, "RT Loop was not running or already stopped.");
    }
    if (motion_interface_ && motion_interface_->isConnected()) {
        LOG_INFO(MODULE_NAME, "Disconnecting motion interface...");
        motion_interface_->disconnect();
    }
}

void MotionManager::emergencyStop() {
    LOG_CRITICAL(MODULE_NAME, "Emergency Stop requested!");
    estop_active_ = true;
    current_mode_.store(RTState::Error); // Or a specific RTState::EStop
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_message_ = "Emergency Stop Active";
        current_error_code_ = 1; // Example E-Stop error code
    }
    
    if (motion_interface_) { // Check if interface exists
        motion_interface_->emergencyStop();
    }
    command_queue_.clear(); // Clear pending commands
    has_active_rt_command_ = false; // Invalidate any held command
    LOG_CRITICAL(MODULE_NAME, "Emergency Stop processed.");
}

void MotionManager::reset() {
    LOG_INFO(MODULE_NAME, "Reset requested.");
    // If the RT loop is running, it should handle state changes.
    // If called externally while loop is stopped, we can set initial states.
    bool loop_was_running = running_.load();
    if (loop_was_running) {
         LOG_WARN(MODULE_NAME, "Reset called while RT loop is running. Loop will pick up reset flags.");
    }

    estop_active_ = false;
    command_queue_.clear();
    // Feedback queue could also be cleared depending on desired behavior
    // feedback_queue_.clear(); 
    has_active_rt_command_ = false;
    current_rt_command_point_.command = RobotCommandFrame{}; // Reset to default/zero command

    if (motion_interface_) {
        motion_interface_->reset(); // Reset the underlying hardware/simulator interface
    }

    current_mode_.store(RTState::Idle); // Reset to Idle
    {
        std::lock_guard<std::mutex> lock(status_mutex_);
        current_status_message_ = "System Reset";
        current_error_code_ = 0;
    }
    LOG_INFO(MODULE_NAME, "MotionManager reset complete.");
}

bool MotionManager::enqueueCommand(const TrajectoryPoint& cmd_point) {
    if (!command_queue_.try_push(cmd_point)) {
        LOG_WARN(MODULE_NAME, "Command queue overflow! Failed to enqueue new command.");
        return false;
    }
    LOG_DEBUG_F(MODULE_NAME, "Enqueued command point. Queue size: %zu", command_queue_.size());
    return true;
}

std::optional<TrajectoryPoint> MotionManager::dequeueFeedback() {
    TrajectoryPoint frame;
    if (feedback_queue_.try_pop(frame)) {
        LOG_DEBUG_F(MODULE_NAME, "Dequeued command point. Queue size: %zu", command_queue_.size());
        return frame;
    }
    return std::nullopt;
}

std::size_t MotionManager::getCommandQueueSize() const {
    return command_queue_.size();
}
std::size_t MotionManager::getFeedbackQueueSize() const {
    return feedback_queue_.size();
}

RTState MotionManager::getCurrentMode() const {
    return current_mode_.load();
}




// --- Private Methods ---

void MotionManager::rt_cycle_tick() {
    LOG_INFO(MODULE_NAME, "RT thread started.");
    if (!motion_interface_ || !motion_interface_->isConnected()) {
        LOG_ERROR(MODULE_NAME, "Motion interface not available or not connected at RT thread start. Thread exiting.");
        running_ = false; // Ensure loop condition will make it exit
        current_mode_.store(RTState::Error);
        {
            std::lock_guard<std::mutex> lock(status_mutex_);
            current_status_message_ = "RT Error: Interface unavailable.";
        }
        return;
    }

    // Initialize current_rt_command_point_ from actual robot state if possible,
    // or ensure it's a safe "zero" command.
    try {
        LOG_INFO(MODULE_NAME, "Reading initial robot state for hold position command...");
        RobotFeedbackFrame initial_feedback = motion_interface_->readState();
        current_rt_command_point_.command.joint_target = initial_feedback.joint_actual;
        has_active_rt_command_ = true;
        LOG_INFO(MODULE_NAME, "Initial hold command set from robot feedback.");
    } catch (const std::exception& e) {
        LOG_WARN_F(MODULE_NAME, "Could not read initial robot state for hold command: %s. Using zero command.", e.what());
        // current_rt_command_point_ is already zeroed from constructor
        has_active_rt_command_ = true; // Still true, but it's a zero command
    }


    while (!rt_thread_.get_stop_token().stop_requested() && running_.load()) {
        auto cycle_start_time = std::chrono::steady_clock::now();

        if (estop_active_.load()) {
            // In E-Stop, we might still want to read state and push it to feedback,
            // but not process new commands or send motion commands other than what E-Stop implies.
            // For now, E-Stop is handled externally primarily by motion_interface_->emergencyStop().
            // The loop will mostly idle here, or we could send "zero velocity" commands.
            // The critical action of iface_->emergencyStop() is called from the public emergencyStop().
            // Here, we just ensure no new motion commands are processed.
            current_mode_.store(RTState::Error); // Or specific EStop state
            // Potentially read state and push to feedback to show E-Stopped state
            try {
                RobotFeedbackFrame current_fb = motion_interface_->readState();
                TrajectoryPoint estop_fb_point = current_rt_command_point_; // Use last known command context
                estop_fb_point.feedback = current_fb;
                estop_fb_point.header.data_type = WaypointDataType::FULL_FB; // Indicate it's feedback
                estop_fb_point.feedback.rt_state = current_mode_.load();
                 if(!feedback_queue_.try_push(std::move(estop_fb_point))) {
                    LOG_WARN(MODULE_NAME, "Feedback queue full during E-Stop state reporting.");
                }
            } catch (const std::exception& e) {
                LOG_ERROR_F(MODULE_NAME, "Error reading state during E-Stop: %s", e.what());
            }
            sleepUntilNextCycle(cycle_start_time);
            continue; // Skip normal command processing
        }

        // Attempt to get a new command
        TrajectoryPoint new_cmd_point;
        if (command_queue_.try_pop(new_cmd_point)) {
            current_rt_command_point_ = new_cmd_point; // This is the new command to execute
            has_active_rt_command_ = true;
            current_mode_.store(RTState::Moving);
            LOG_DEBUG_F(MODULE_NAME, "RT: Popped new command. Seq: %u, Type: na, %s", 
                        current_rt_command_point_.header.sequence_index, 
                        Axis2Str(current_rt_command_point_.command.joint_target).c_str());
        } else {
            // No new command, process "idle" or "hold position" logic
            current_mode_.store(RTState::Idle);
            LOG_DEBUG(MODULE_NAME, "RT: Command queue empty. Processing idle/hold.");
            processIdleState();
        }

        // TrajectoryPoint new_fb_point = sendToHALandGetFeedback(current_rt_command_point_); // Process it
        auto new_fb_point_opt = sendToHALandGetFeedback(current_rt_command_point_); // Process it
        if (new_fb_point_opt.has_value()) {
            const TrajectoryPoint& new_fb_point = new_fb_point_opt.value();
            if(!new_fb_point.header.has_error_at_this_point){
                current_rt_command_point_ = new_fb_point;
            }
            // Regardless of readState success/failure (if it didn't throw out), push to feedback queue.
            // The header.has_error_at_this_point and feedback.rt_state will indicate issues.
            if (!feedback_queue_.try_push(new_fb_point)) {
                LOG_WARN_F(MODULE_NAME, "RT: Feedback queue full! Failed to push feedback for seq: %u.", new_fb_point.header.sequence_index);
            }
        }

        sleepUntilNextCycle(cycle_start_time);
    }
    LOG_INFO(MODULE_NAME, "RT thread finishing.");
}


std::optional <TrajectoryPoint> MotionManager::sendToHALandGetFeedback(const TrajectoryPoint& point_to_update) {

    // `point_to_update` is typically `current_rt_command_point_`
    LOG_DEBUG_F(MODULE_NAME, "RT: Processing command. Seq: %u, Type: %d, %s ",
                point_to_update.header.sequence_index, static_cast<int>(point_to_update.header.motion_type),
                Axis2Str(point_to_update.command.joint_target).c_str()
            
            );

    TrajectoryPoint updated_point = point_to_update;
    // Send the command part of the point_to_update
    bool sent_ok = false;
    try {
        sent_ok = motion_interface_->sendCommand(point_to_update.command);
    } catch (const std::exception& e) {
        LOG_ERROR_F(MODULE_NAME, "RT: Exception during sendCommand: %s", e.what());
        current_mode_.store(RTState::Error);
        { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Send command error"; current_error_code_ = 2; }
        // Populate feedback with error
        updated_point.feedback.rt_state = current_mode_.load();
        updated_point.header.has_error_at_this_point = true;
        if(!feedback_queue_.try_push(point_to_update)) { // Push the point with error state
            LOG_WARN(MODULE_NAME, "RT: Feedback queue full when pushing error feedback for send failure.");
        }
       return std::nullopt;
    }



    if (!sent_ok) {
        LOG_WARN(MODULE_NAME, "RT: motion_interface_->sendCommand failed for seq: " + std::to_string(point_to_update.header.sequence_index));
        current_mode_.store(RTState::Error); // Or some other state like "InterfaceBusy"
        { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Send command failed (rejected by interface)"; current_error_code_ = 3; }
        updated_point.feedback.rt_state = current_mode_.load();
        updated_point.header.has_error_at_this_point = true;
        if(!feedback_queue_.try_push(updated_point)) {
            LOG_WARN(MODULE_NAME, "RT: Feedback queue full when pushing error feedback for send rejection.");
        }
        return std::nullopt;
    }


    // Read feedback from the interface
    try {
        updated_point.feedback = motion_interface_->readState();
        // Ensure the RTState from MM is also part of the feedback point for consistency
        updated_point.feedback.rt_state = current_mode_.load(); // current_mode_ could be Moving or Error if sendCommand caused it
        updated_point.header.is_target_reached_for_this_point = point_to_update.feedback.target_reached; // Assuming this mapping
        LOG_INFO_F(MODULE_NAME, "RT: Feedback received. Actual%s", Axis2Str(updated_point.feedback.joint_actual).c_str());
        //;
               
    } catch (const std::exception& e) {
        LOG_ERROR_F(MODULE_NAME, "RT: Exception during readState: %s", e.what());
        current_mode_.store(RTState::Error);
         { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Read state error"; current_error_code_ = 4; }
        // Populate feedback with error
        updated_point.feedback.rt_state = current_mode_.load(); // Error state
        updated_point.header.has_error_at_this_point = true;
        // Still try to push the point with the error indication
    }
    
    
    return updated_point; // No new command to return, as we processed the point_to_update
}


void MotionManager::processIdleState() {
    // No new commands in the queue.
    // We should send the "current_rt_command_point_.command" to effectively "hold position"
    // or whatever the last command implied, if has_active_rt_command_ is true.
    // If no command was ever active, it sends a zero/default command.
    LOG_DEBUG(MODULE_NAME, "RT: Idle state. Sending current/hold command.");

    if (!has_active_rt_command_) {
        // This case should ideally be rare if current_rt_command_point_ is initialized properly
        // from robot state or to zero.
        LOG_WARN(MODULE_NAME, "RT: Idle, but no active command to hold. Sending zero command.");
        // current_rt_command_point_.command is already zeroed/default.
    }
    
    // Create a temporary point for this idle feedback, using current_rt_command_point_ as context
    TrajectoryPoint idle_feedback_point = current_rt_command_point_; 
    // Mark it as feedback or a status update, not a new command being processed
    idle_feedback_point.header.data_type = WaypointDataType::FULL_FB; 
    idle_feedback_point.header.motion_type = MotionType::JOINT; // Or reflect last motion type
    idle_feedback_point.header.sequence_index = 0; // Or a special sequence for idle ticks
    idle_feedback_point.header.trajectory_id = 0; // Or a special ID

    // Send the "hold" command
    bool sent_ok = false;
    try {
       sent_ok = motion_interface_->sendCommand(current_rt_command_point_.command);
    } catch (const std::exception& e) {
        LOG_ERROR_F(MODULE_NAME, "RT: Exception during sendCommand (idle/hold): %s", e.what());
        current_mode_.store(RTState::Error);
        { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Send hold command error"; current_error_code_ = 5; }
        idle_feedback_point.feedback.rt_state = current_mode_.load();
        idle_feedback_point.header.has_error_at_this_point = true;
         if(!feedback_queue_.try_push(std::move(idle_feedback_point))) {
            LOG_WARN(MODULE_NAME, "RT: Feedback queue full when pushing error feedback for idle send failure.");
        }
        return;
    }

    if (!sent_ok) {
        LOG_WARN(MODULE_NAME, "RT: motion_interface_->sendCommand (idle/hold) failed.");
        current_mode_.store(RTState::Error);
         { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Send hold command failed (rejected)"; current_error_code_ = 6; }
        idle_feedback_point.feedback.rt_state = current_mode_.load();
        idle_feedback_point.header.has_error_at_this_point = true;
        if(!feedback_queue_.try_push(std::move(idle_feedback_point))) {
            LOG_WARN(MODULE_NAME, "RT: Feedback queue full when pushing error feedback for idle send rejection.");
        }
        return;
    }

    // Read current state
    try {
        idle_feedback_point.feedback = motion_interface_->readState();
        idle_feedback_point.feedback.rt_state = current_mode_.load(); // Should be Idle
        LOG_DEBUG(MODULE_NAME, "RT: Idle feedback received.");
    } catch (const std::exception& e) {
        LOG_ERROR_F(MODULE_NAME, "RT: Exception during readState (idle): %s", e.what());
        current_mode_.store(RTState::Error);
        { std::lock_guard<std::mutex> lock(status_mutex_);
          current_status_message_ = "Read state error (idle)"; current_error_code_ = 7; }
        idle_feedback_point.feedback.rt_state = current_mode_.load(); // Error state
        idle_feedback_point.header.has_error_at_this_point = true; // Mark this specific feedback point as problematic
    }

    if (!feedback_queue_.try_push(std::move(idle_feedback_point))) {
        LOG_WARN(MODULE_NAME, "RT: Feedback queue full! Failed to push idle feedback.");
    }
}

std::string MotionManager::getActiveInterfaceName() const {
    //   motion_interface_   ,    .
    //          .
    //  isConnected()   ,     .
    //  , ,      const   
    //      MotionManager,  IMotionInterface::getInterfaceName() const.
    if (motion_interface_) { // ,    
        //    motion_interface_->isConnected(),     ""
        return motion_interface_->getInterfaceName();
    }
    return "NoActiveInterface";
}

void MotionManager::sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - cycle_start_time;
    if (elapsed < cycle_period_) {
        std::this_thread::sleep_until(cycle_start_time + cycle_period_);
    } else {
        // Cycle overrun
        LOG_WARN_F(MODULE_NAME, "RT cycle overrun! Elapsed: %lld us, Period: %lld us",
                   std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count(),
                   std::chrono::duration_cast<std::chrono::microseconds>(cycle_period_).count());
        // Potentially increment an overrun counter
    }
}

} // namespace RDT