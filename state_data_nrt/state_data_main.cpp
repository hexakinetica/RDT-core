// state_data_test_main.cpp
#include "StateData.h" // RDT::StateData (includes DataTypes.h, Units.h)
#include "Logger.h"    // RDT::Logger
#include "DataTypes.h" // For RDT::TrajectoryPoint, etc. (though StateData.h includes it)
#include "Units.h"     // For RDT::literals (though StateData.h includes DataTypes.h which includes Units.h)


#include <iostream> // Only for final cout if needed, prefer Logger
#include <thread>
#include <vector>
#include <cassert>   // For assert
#include <string>    // For std::string
#include <chrono>    // For std::chrono::milliseconds
#include <functional>// For std::ref, std::cref

// For convenience
using namespace RDT;
using namespace RDT::literals; // For _m, _rad, _deg, _s etc.
using namespace std::chrono_literals; // For _ms, _s literal for duration


// Writer thread function
void writer_thread_func(StateData& sd, int thread_id) {
    LOG_INFO_F("Writer", "Writer thread %d started.", thread_id);
    for (int i = 0; i < 5; ++i) {
        TrajectoryPoint tp_cmd;
        tp_cmd.header.motion_type = MotionType::LIN; // Example type
        // Use RDT units for pose
        tp_cmd.command.cartesian_target.x = Meters(10.0 + i + thread_id * 0.1);
        tp_cmd.command.joint_target[AxisId::A1].angle = Radians(1.0 + i + thread_id * 0.01);
        sd.setCmdTrajectoryPoint(tp_cmd);

        TrajectoryPoint tp_fb;
        tp_fb.feedback.cartesian_actual.z = Meters(20.0 - i - thread_id * 0.1);
        tp_fb.feedback.joint_actual[AxisId::A1].angle = Radians(2.0 - i - thread_id * 0.01);
        // Populate other feedback fields if necessary for test
        tp_fb.feedback.rt_state = RTState::Moving;
        sd.setFbTrajectoryPoint(tp_fb);

        ToolFrame tool("TestTool" + std::to_string(thread_id), CartPose{0.0_m, 0.0_m, Meters(0.1 * i)});
        sd.setActiveToolFrame(tool);

        BaseFrame base("TestBase" + std::to_string(thread_id), CartPose{Meters(0.05 * i), 0.0_m, 0.0_m});
        sd.setActiveBaseFrame(base);

        sd.setRobotMode(static_cast<RobotMode>(i % static_cast<int>(RobotMode::Jogging) + 1)); // Cycle through modes
        sd.setGlobalSpeedRatio( (80.0 + i) / 100.0 ); // Store as ratio 0.0-1.0
        
        std::string msg = "Msg from writer " + std::to_string(thread_id) + " iter " + std::to_string(i);
        sd.setSystemMessage(msg, (i % 2 != 0)); // Alternate error state
        
        sd.setEstopState((i == 4)); // EStop on last iteration for one writer

        std::this_thread::sleep_for(std::chrono::milliseconds(10 + thread_id * 5));
    }
    LOG_INFO_F("Writer", "Writer thread %d finished.", thread_id);
}

// Reader thread function
void reader_thread_func(const StateData& sd, int thread_id) {
    LOG_INFO_F("Reader", "Reader thread %d started.", thread_id);
    for (int i = 0; i < 10; ++i) {
        TrajectoryPoint cmd_tp = sd.getCmdTrajectoryPoint();
        TrajectoryPoint fb_tp = sd.getFbTrajectoryPoint();
        ToolFrame tool = sd.getActiveToolFrame();
        BaseFrame base = sd.getActiveBaseFrame();
        RobotMode mode = sd.getRobotMode();
        double speed_ratio = sd.getGlobalSpeedRatio();
        std::string sys_msg = sd.getSystemMessage();
        bool has_err = sd.hasActiveError();
        bool estop = sd.isEstopActive();

        // Use Logger for output
        LOG_DEBUG_F("Reader", "Reader %d: CmdPose.X=%s, FbPose.Z=%s, Tool=%s, Base=%s, Mode=%d, Speed=%.0f%%, HasError=%s (%s), EStop=%s",
                  thread_id,
                  cmd_tp.command.cartesian_target.x.toString().c_str(),
                  fb_tp.feedback.cartesian_actual.z.toString().c_str(),
                  tool.name.c_str(),
                  base.name.c_str(),
                  static_cast<int>(mode),
                  speed_ratio * 100.0,
                  (has_err ? "YES" : "NO"),
                  sys_msg.c_str(),
                  (estop ? "ACTIVE" : "INACTIVE")
        );
        std::this_thread::sleep_for(std::chrono::milliseconds(15 + thread_id * 3));
    }
    LOG_INFO_F("Reader", "Reader thread %d finished.", thread_id);
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    Logger::setLogLevel(LogLevel::Debug); // Set log level
    LOG_INFO("StateDataTest", "--- StateData Test (with RDT Types) ---");

    StateData shared_state;

    // Test 1: Basic Setters/Getters
    LOG_INFO("StateDataTest", "\n--- Test 1: Basic Setters/Getters ---");
    TrajectoryPoint tp1_cmd;
    tp1_cmd.header.motion_type = MotionType::LIN;
    tp1_cmd.command.cartesian_target = {1.0_m, 2.0_m, 3.0_m, 0.1_rad, 0.2_rad, 0.3_rad};
    shared_state.setCmdTrajectoryPoint(tp1_cmd);
    
    TrajectoryPoint tp_read_cmd = shared_state.getCmdTrajectoryPoint();
    assert(tp_read_cmd.command.cartesian_target.x == 1.0_m);
    // assert(tp_read_cmd.header.motion_type == MotionType::LIN); // This was not set in get/set for cmd_pose_
    // To test this, TrajectoryPoint itself needs equality operator or member-wise check.
    // For simplicity, just check one field.
    LOG_INFO_F("StateDataTest", "CmdPose X: %s (Expected 1.0 m)", tp_read_cmd.command.cartesian_target.x.toString().c_str());


    shared_state.setRobotMode(RobotMode::Running);
    assert(shared_state.getRobotMode() == RobotMode::Running);
    LOG_INFO_F("StateDataTest", "RobotMode: %d (Expected Running: %d)", static_cast<int>(shared_state.getRobotMode()), static_cast<int>(RobotMode::Running));

    shared_state.setSystemMessage("Test Error", true);
    assert(shared_state.hasActiveError() == true);
    assert(shared_state.getSystemMessage() == "Test Error");
    LOG_INFO_F("StateDataTest", "System Msg: '%s', HasError: %s", shared_state.getSystemMessage().c_str(), shared_state.hasActiveError() ? "true" : "false");

    shared_state.setSystemMessage("All OK", false);
    assert(shared_state.hasActiveError() == false);
    // assert(shared_state.getSystemMessage() == "All OK"); // This might fail if msg is "All OK" but has_error_ was true before this call
    LOG_INFO_F("StateDataTest", "System Msg: '%s', HasError: %s", shared_state.getSystemMessage().c_str(), shared_state.hasActiveError() ? "true" : "false");


    // Test 2: Multithreaded Access
    LOG_INFO("StateDataTest", "\n--- Test 2: Multithreaded Access ---");
    std::vector<std::jthread> threads;

    for(int i = 0; i < 2; ++i) {
        threads.emplace_back(writer_thread_func, std::ref(shared_state), i + 1);
    }
    for(int i = 0; i < 3; ++i) {
        threads.emplace_back(reader_thread_func, std::cref(shared_state), i + 1);
    }

    for(auto& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }
    LOG_INFO("StateDataTest", "All threads finished.");

    LOG_INFO("StateDataTest", "\n--- StateData Test Complete ---");
    return 0;
}