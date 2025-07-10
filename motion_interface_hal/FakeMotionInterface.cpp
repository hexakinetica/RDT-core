// FakeMotionInterface.cpp
#include "FakeMotionInterface.h"
#include <cmath> // For std::abs

namespace RDT {

FakeMotionInterface::FakeMotionInterface() {
    LOG_INFO(MODULE_NAME, "FakeMotionInterface created.");
    reset(); // Initialize to a clean state
}

FakeMotionInterface::~FakeMotionInterface() {
    LOG_INFO(MODULE_NAME, "FakeMotionInterface destroyed.");
    if (connected_) {
        disconnect();
    }
}

//  ,  . ,      .
void FakeMotionInterface::reset_impl() {
    //  : std::lock_guard<std::mutex> lock(state_mutex_);
    LOG_INFO(MODULE_NAME, "Resetting state (impl).");
    current_feedback_state_.joint_actual[AxisId::A3].angle=1.5708_rad; //  
    // 'connected_'        (connect/disconnect)
    //   reset_impl,      ,
    //    reset_impl    rt_state.
    current_feedback_state_.rt_state = RTState::Idle;
    last_command_time_ = std::chrono::steady_clock::now();
}

bool FakeMotionInterface::connect() {
    std::lock_guard<std::mutex> lock(state_mutex_); //  
    if (connected_) {
        LOG_WARN(MODULE_NAME, "Already connected.");
        return true;
    }
    LOG_INFO(MODULE_NAME, "Connecting...");
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    
    connected_ = true; //  connected_   reset_impl
    reset_impl();      //  reset_impl,    
    // current_feedback_state_.rt_state   Idle  reset_impl
    
    LOG_INFO(MODULE_NAME, "Connected successfully.");
    return true;
}

void FakeMotionInterface::disconnect() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!connected_) {
        LOG_WARN(MODULE_NAME, "Not connected, nothing to disconnect.");
        return;
    }
    LOG_INFO(MODULE_NAME, "Disconnecting...");
    connected_ = false; //  connected_
    reset_impl();      //   reset_impl   
                       //    rt_state,  reset_impl   .
                       //  ,  reset_impl.
    // current_feedback_state_.rt_state = RTState::Idle; //    reset_impl
    LOG_INFO(MODULE_NAME, "Disconnected.");
}


//   reset     
void FakeMotionInterface::reset() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    reset_impl(); //  
}

bool FakeMotionInterface::isConnected() const {
    // No lock needed for simple bool read if updates are locked,
    // but for consistency with other methods, or if more complex logic is added:
    // std::lock_guard<std::mutex> lock(state_mutex_);
    return connected_;
}

bool FakeMotionInterface::sendCommand(const RobotCommandFrame& cmd) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!connected_) {
        LOG_ERROR(MODULE_NAME, "Cannot send command: Not connected.");
        return false;
    }

    last_command_time_ = std::chrono::steady_clock::now();

    // Simplistic simulation: command directly influences the "actual" state after a tiny delay.
    // More advanced simulation would involve a trajectory generator.
    current_feedback_state_.joint_actual     = cmd.joint_target; // This is a simplification
    
    // Simulate motion
    if (cmd.joint_target!= prev_joint_target_ ) { // Basic check if it's not a zero command
         current_feedback_state_.rt_state = RTState::Moving;
    } else {
         current_feedback_state_.rt_state = RTState::Idle; // Or based on speed_ratio
    }
    current_feedback_state_.current_speed_ratio = cmd.speed_ratio;
    current_feedback_state_.target_reached = true; // Simulate instant reach for simplicity
    current_feedback_state_.arrival_time = std::chrono::steady_clock::now();
    current_feedback_state_.path_deviation = 0.0_m;
    prev_joint_target_=current_feedback_state_.joint_actual;
    return true;
}

RobotFeedbackFrame FakeMotionInterface::readState() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!connected_) {
        LOG_WARN(MODULE_NAME, "Reading state while not connected. Returning default/last known state.");
        // Optionally, set rt_state to an "offline" or error state
        // For now, just return the current state which might reflect disconnected
    }
    // Simulate some state evolution if needed, e.g., if rt_state is Moving, update a little
    if (current_feedback_state_.rt_state == RTState::Moving) {
        // A very simple "movement stops after a short time" simulation
        auto time_since_last_cmd = std::chrono::steady_clock::now() - last_command_time_;
        if (time_since_last_cmd > std::chrono::milliseconds(500)) { // Example: stops after 500ms
            // current_feedback_state_.rt_state = RTState::Idle;
            // current_feedback_state_.current_speed_ratio = 0.0;
        }
    }
    return current_feedback_state_;
}

void FakeMotionInterface::emergencyStop() {
    std::lock_guard<std::mutex> lock(state_mutex_);
    LOG_WARN(MODULE_NAME, "Emergency Stop activated!");
    current_feedback_state_.rt_state = RTState::Error; // Or a specific EStop state
    current_feedback_state_.current_speed_ratio = 0.0;
    
    // Set all joint angles/velocities to zero and engage brakes for simulation
    std::array<Radians, ROBOT_AXES_COUNT> zero_angles; // Default constructs to 0.0_rad
    current_feedback_state_.joint_actual.fromAngleArray(zero_angles);
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        current_feedback_state_.joint_actual.at(i).velocity = 0.0_rad_s;
        current_feedback_state_.joint_actual.at(i).acceleration = 0.0_rad_s2;
        current_feedback_state_.joint_actual.at(i).brake_engaged = true; // Simulate brake engagement
    }
}

std::string FakeMotionInterface::getInterfaceName() const {
    return "FakeMotionSimulator";
}

} // namespace RDT
