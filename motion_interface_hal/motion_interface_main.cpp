// main.cpp (Test for Motion Interfaces)
#include "IMotionInterfaces.h"
#include "FakeMotionInterface.h"
#include "UDPMotionInterface.h"
#include "UDPTransport.h" // For creating transport for UDPMotionInterface
#include "Logger.h"
#include "DataTypes.h" // For command/feedback structs and units

#include <iostream>
#include <vector>
#include <thread> // For std::this_thread::sleep_for
#include <memory> // For std::unique_ptr

// Helper to print RobotFeedbackFrame (same as in MotionController test)
void printRobotFeedback(const RDT::RobotFeedbackFrame& feedback, const std::string& interface_name) {
    LOG_INFO_F("PrintFeedback", "State from [%s]:", interface_name.c_str());
    LOG_INFO_F("PrintFeedback", "  RT State: %d", static_cast<int>(feedback.rt_state)); // Assuming no ostream op for RTState yet
    LOG_INFO_F("PrintFeedback", "  Target Reached: %s", feedback.target_reached ? "Yes" : "No");
    LOG_INFO_F("PrintFeedback", "  Speed Ratio: %.2f %%", feedback.current_speed_ratio * 100.0);
    LOG_INFO_F("PrintFeedback", "  Joint Actual (deg): %s", feedback.joint_actual.toJointPoseString().c_str());
    LOG_INFO_F("PrintFeedback", "  Path Deviation: %s", feedback.path_deviation.toString().c_str());
}

void testMotionInterface(RDT::IMotionInterface& interface) {
    using namespace RDT;
    using namespace RDT::literals;

    LOG_INFO_F("TestInterface", "\n--- Testing Interface: %s ---", interface.getInterfaceName().c_str());

    if (interface.connect()) {
        LOG_INFO_F("TestInterface", "%s connected successfully.", interface.getInterfaceName().c_str());

        LOG_INFO("TestInterface", "Reading initial state...");
        try {
            RobotFeedbackFrame initial_state = interface.readState();
            printRobotFeedback(initial_state, interface.getInterfaceName());
        } catch (const std::exception& e) {
            LOG_ERROR_F("TestInterface", "Error reading initial state: %s", e.what());
        }

        LOG_INFO("TestInterface", "Sending a PTP command...");
        RobotCommandFrame cmd1;
        // For a PTP, joint_target might be calculated by a planner or be primary
        std::array<Radians, ROBOT_AXES_COUNT> joint_angles_cmd1 = {
            (10.0_deg).toRadians(), (20.0_deg).toRadians(), (30.0_deg).toRadians(),
            0.0_rad, 0.0_rad, 0.0_rad
        };
        cmd1.joint_target.fromAngleArray(joint_angles_cmd1);


        if (interface.sendCommand(cmd1)) {
            LOG_INFO("TestInterface", "PTP command sent. Waiting for mock execution/feedback...");
            std::this_thread::sleep_for(std::chrono::milliseconds(250)); // Simulate time for command to process
            try {
                RobotFeedbackFrame state_after_cmd1 = interface.readState();
                printRobotFeedback(state_after_cmd1, interface.getInterfaceName());
            } catch (const std::exception& e) {
                LOG_ERROR_F("TestInterface", "Error reading state after PTP command: %s", e.what());
            }
        } else {
            LOG_ERROR("TestInterface", "Failed to send PTP command.");
        }

        LOG_INFO("TestInterface", "Sending Emergency Stop...");
        interface.emergencyStop();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
         try {
            RobotFeedbackFrame state_after_estop = interface.readState();
            printRobotFeedback(state_after_estop, interface.getInterfaceName());
        } catch (const std::exception& e) {
            LOG_ERROR_F("TestInterface", "Error reading state after E-Stop: %s", e.what());
        }


        LOG_INFO("TestInterface", "Sending Reset...");
        interface.reset();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        try {
            RobotFeedbackFrame state_after_reset = interface.readState();
            printRobotFeedback(state_after_reset, interface.getInterfaceName());
        } catch (const std::exception& e) {
            LOG_ERROR_F("TestInterface", "Error reading state after reset: %s", e.what());
        }

        interface.disconnect();
        LOG_INFO_F("TestInterface", "%s disconnected.", interface.getInterfaceName().c_str());
    } else {
        LOG_ERROR_F("TestInterface", "Failed to connect to %s.", interface.getInterfaceName().c_str());
    }
}


int main(int argc, char *argv[]) {
    RDT::Logger::setLogLevel(RDT::LogLevel::Debug);
    LOG_INFO("MainApp", "Motion Interface Test Application Started.");

    // Test FakeMotionInterface
    RDT::FakeMotionInterface fake_if;
    testMotionInterface(fake_if);

    // Test UDPMotionInterface
    // For this to work meaningfully, a UDP echo server or the actual robot needs to be running.
    // The echo server should listen on local_port and send replies to remote_port.
    // This test will likely show timeouts or errors for receive() if no peer is present.
    LOG_INFO("MainApp", "\nAttempting to test UDPMotionInterface...");
    LOG_WARN("MainApp", "NOTE: UDPMotionInterface test requires a listening UDP peer on configured ports.");
    LOG_WARN("MainApp", "Otherwise, connection may succeed but send/receive will likely fail/timeout.");

    try {
        RDT::NetworkConfig udp_config = {
            50001,             // Local port to bind to for receiving
            "127.0.0.1",       // Remote robot IP (loopback for testing)
            50002,             // Remote robot port to send to
            4096,              // Max packet size
            500                // Receive timeout ms (increased for testing)
        };
        auto udp_transport = std::make_unique<RDT::UDPTransport>(udp_config);
        RDT::UDPMotionInterface udp_if(std::move(udp_transport));
        testMotionInterface(udp_if);
    } catch (const std::exception& e) {
        LOG_CRITICAL_F("MainApp", "Failed to initialize or run UDPMotionInterface test: %s", e.what());
        LOG_WARN("MainApp", "This might be due to network issues or no UDP peer available at 127.0.0.1:50002 (sending to) / 50001 (listening on).");
    }


    LOG_INFO("MainApp", "Motion Interface Test Application Finished.");
    return 0;
}