// robot_controller_main.cpp
#include "RobotController.h"
#include "StateData.h"
#include "KinematicModel.h"
#include "Logger.h"
#include "Units.h"
#include "RobotConfig.h"

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <vector>

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

// *** ИСПРАВЛЕНИЕ: Объявление helper-функций ДО их первого использования ***
void printAxisSet(const std::string& prefix, const AxisSet& axes);

// Helper to print a snapshot of the system state
void printStateDataSnapshot(const StateData& sd, const std::string& context_msg) {
    LOG_INFO("StateSnapshot", "----------------------------------------------------------");
    LOG_INFO_F("StateSnapshot", "--- %s ---", context_msg.c_str());
    
    TrajectoryPoint fb_tp = sd.getFbTrajectoryPoint();
    
    LOG_INFO_F("StateSnapshot", "Mode: %d | E-Stop: %s | Error: %s ('%s')",
        static_cast<int>(sd.getRobotMode()), 
        sd.isEstopActive() ? "ACTIVE" : "inactive",
        sd.hasActiveError() ? "ACTIVE" : "inactive", 
        sd.getSystemMessage().c_str());

    std::string fault_str = "None";
    if (fb_tp.feedback.fault.type == FaultType::VelocityLimit) {
        fault_str = "Velocity Limit on A" + std::to_string(static_cast<int>(fb_tp.feedback.fault.axis)+1);
    } else if (fb_tp.feedback.fault.type == FaultType::PositionLimit) {
        fault_str = "Position Limit on A" + std::to_string(static_cast<int>(fb_tp.feedback.fault.axis)+1);
    }
    LOG_INFO_F("StateSnapshot", "Last Fault: %s", fault_str.c_str());

    printAxisSet("Feedback Joints", fb_tp.feedback.joint_actual);
    LOG_INFO("StateSnapshot", "----------------------------------------------------------");
}

// *** ИСПРАВЛЕНИЕ: Определение helper-функции ***
void printAxisSet(const std::string& prefix, const AxisSet& axes) {
    std::string s = prefix + ": [ ";
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        s += "A" + std::to_string(i + 1) + ":" + axes.at(i).angle.toDegrees().toString(2) + (i == 5 ? "" : ", ");
    }
    s += " ]";
    LOG_INFO("StateSnapshot", s);
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    Logger::setLogLevel(LogLevel::Info);
    LOG_INFO("UltimateTest", "--- ULTIMATE ROBOT CONTROLLER PIPELINE TEST ---");

    // --- 1. SETUP ---
    InterfaceConfig hw_config;
    hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::EtherCAT;
    hw_config.simulation_initial_joints.fromAngleArray({0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad});

    ControllerConfig ctrl_config;
    ctrl_config.motion_manager_cycle_ms = 20;

    auto state_data = std::make_shared<StateData>();
    KinematicModel kuka_model = KinematicModel::createKR6R900();
    
    RobotController robot_controller(hw_config, ctrl_config, kuka_model, state_data);

    TrajectoryPoint initial_cmd;
    initial_cmd.command.joint_target = kuka_model.getHomePositionJoints();
    if (!robot_controller.initialize(initial_cmd)) {
        LOG_CRITICAL("UltimateTest", "Initialization failed!");
        return 1;
    }
    
    std::jthread feedback_consumer_thread([&state_data](std::stop_token stoken) {
        LOG_INFO("FeedbackConsumer", "Feedback consumer thread started.");
        while (!stoken.stop_requested()) {
            // В этом тесте мы не используем feedback_queue, а смотрим на StateData
            // printStateDataSnapshot(*state_data, "Live Update");
            std::this_thread::sleep_for(500ms); 
        }
    });
    
    std::this_thread::sleep_for(200ms);

    // --- 2. VELOCITY LIMIT TEST ---
    LOG_INFO("UltimateTest", "\n\n--- TEST 1: VELOCITY LIMIT ---");
    LOG_WARN("UltimateTest", "Sending a fast move (A1 -> 20 deg). Expecting Velocity Limit warnings in feedback.");
    TrajectoryPoint fast_pt;
    fast_pt.header.motion_type = MotionType::PTP;
    fast_pt.command.joint_target[AxisId::A1].angle = (20.0_deg).toRadians();
    // *** ИСПРАВЛЕНИЕ: Проверяем возвращаемое значение ***
    if (!robot_controller.executeMotionToTarget(fast_pt)) { LOG_ERROR("UltimateTest", "Test 1 FAILED: executeMotionToTarget returned false."); }
    while(robot_controller.isMotionTaskActive()) { std::this_thread::sleep_for(100ms); }
    printStateDataSnapshot(*state_data, "After fast move");

    // --- 3. POSITION LIMIT TEST ---
    LOG_INFO("UltimateTest", "\n\n--- TEST 2: POSITION LIMIT ---");
    LOG_WARN("UltimateTest", "Sending a move outside A1 limits (limit is +/- 170). Expecting ERROR state.");
    TrajectoryPoint illegal_pt;
    illegal_pt.header.motion_type = MotionType::PTP;
    illegal_pt.command.joint_target[AxisId::A1].angle = (180.0_deg).toRadians();
    // *** ИСПРАВЛЕНИЕ: Проверяем возвращаемое значение ***
    if (!robot_controller.executeMotionToTarget(illegal_pt)) { LOG_INFO("UltimateTest", "Test 2: executeMotionToTarget correctly returned false on illegal point."); }
    std::this_thread::sleep_for(200ms);
    if (robot_controller.getInternalControllerState() != ControllerState::Error) {
        LOG_ERROR("UltimateTest", "Test 2 FAILED: Controller did not enter Error state.");
    } else {
        LOG_INFO("UltimateTest", "Test 2 PASSED: Controller correctly entered Error state.");
    }
    printStateDataSnapshot(*state_data, "After illegal move attempt");

    // --- 4. RESET TEST ---
    LOG_INFO("UltimateTest", "\n\n--- TEST 3: RESET AFTER ERROR ---");
    robot_controller.reset();
    std::this_thread::sleep_for(200ms);
     if (robot_controller.getInternalControllerState() != ControllerState::Idle) {
         LOG_ERROR("UltimateTest", "Test 3 FAILED: Controller is not Idle after reset.");
    } else {
        LOG_INFO("UltimateTest", "Test 3 PASSED: Controller is Idle after reset.");
    }
    printStateDataSnapshot(*state_data, "After reset");

    // --- 5. MODE SWITCHING TEST ---
    LOG_INFO("UltimateTest", "\n\n--- TEST 4: MODE SWITCHING ---");
    TrajectoryPoint sim_only_move;
    sim_only_move.header.motion_type = MotionType::PTP;
    sim_only_move.command.joint_target[AxisId::A2].angle = (-45.0_deg).toRadians();
    // *** ИСПРАВЛЕНИЕ: Проверяем возвращаемое значение ***
    if (!robot_controller.executeMotionToTarget(sim_only_move)) { LOG_ERROR("UltimateTest", "Test 4 FAILED: Could not execute sim_only_move."); }
    while(robot_controller.isMotionTaskActive()) { std::this_thread::sleep_for(100ms); }
    LOG_INFO("UltimateTest", "Simulation moved to a new position.");
    printStateDataSnapshot(*state_data, "After sim-only move");

    LOG_INFO("UltimateTest", "Attempting to switch to Realtime (should fail)...");
    auto switch_res = robot_controller.requestModeSwitch(MasterHardwareInterface::ActiveMode::Realtime);
    if (switch_res == RobotController::SwitchRequestResult::NotInSync) {
        LOG_INFO("UltimateTest", "Switch failed as expected. Forcing sync...");
        robot_controller.forceSync();
        LOG_INFO("UltimateTest", "Retrying switch...");
        switch_res = robot_controller.requestModeSwitch(MasterHardwareInterface::ActiveMode::Realtime);
    }
    if (switch_res == RobotController::SwitchRequestResult::Success) {
        LOG_INFO("UltimateTest", "Test 4 PASSED: Successfully switched to Realtime mode.");
    } else {
        LOG_ERROR("UltimateTest", "Test 4 FAILED: Could not switch to Realtime mode.");
    }
    printStateDataSnapshot(*state_data, "After switching to Realtime");

    // --- 6. E-STOP TEST ---
    LOG_INFO("UltimateTest", "\n\n--- TEST 5: E-STOP ---");
    LOG_INFO("UltimateTest", "Starting a long move in Realtime and triggering E-Stop...");
    TrajectoryPoint long_move;
    long_move.header.motion_type = MotionType::PTP;
    long_move.command.joint_target[AxisId::A3].angle = (90.0_deg).toRadians();
    // *** ИСПРАВЛЕНИЕ: Проверяем возвращаемое значение ***
    if (!robot_controller.executeMotionToTarget(long_move)) { LOG_ERROR("UltimateTest", "Test 5 FAILED: Could not execute long_move."); }
    std::this_thread::sleep_for(300ms);
    LOG_CRITICAL("UltimateTest", ">>> TRIGGERING E-STOP NOW <<<");
    robot_controller.emergencyStop();
    std::this_thread::sleep_for(200ms);
    if (robot_controller.getInternalControllerState() == ControllerState::Error && state_data->isEstopActive()) {
        LOG_INFO("UltimateTest", "Test 5 PASSED: Controller is in E-Stop state.");
    } else {
        LOG_ERROR("UltimateTest", "Test 5 FAILED: Controller did not enter E-Stop state correctly.");
    }
    printStateDataSnapshot(*state_data, "After E-Stop");

    // --- 7. FINAL ---
    LOG_INFO("UltimateTest", "\n--- All tests complete. ---");
    feedback_consumer_thread.request_stop();
    if (feedback_consumer_thread.joinable()) {
        feedback_consumer_thread.join();
    }
    return 0;
}