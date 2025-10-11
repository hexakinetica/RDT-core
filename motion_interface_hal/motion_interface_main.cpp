// master_hw_iface_main.cpp
#include "MasterHardwareInterface.h"
#include "RobotConfig.h"
#include "Logger.h"
#include "Units.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <cstdlib> // For std::system

using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

// --- Configuration Flag ---
// Set to true to automatically launch the python emulator from this C++ test.
// Set to false if you want to run the python script manually.
constexpr bool LAUNCH_EMULATOR_AUTOMATICALLY = true;

void printAxisSet(const std::string& prefix, const AxisSet& axes) {
    std::ostringstream ss;
    ss << prefix << ": [ ";
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        ss << "A" << (i + 1) << ": " << axes.at(i).angle.toDegrees().toString(2) << (i == 5 ? "" : ", ");
    }
    ss << " ]";
    LOG_INFO("TestMain", ss.str());
}

int main() {
    Logger::setLogLevel(LogLevel::Info);
    LOG_INFO("TestMain", "--- MasterHardwareInterface Comprehensive Test (UDP) ---");


    const bool LAUNCH_EMULATOR_AUTOMATICALLY = true;
 
    // --- 0. AUTO-LAUNCH EMULATOR (New Feature) ---
    if (LAUNCH_EMULATOR_AUTOMATICALLY) {
        LOG_INFO("TestMain", "Automatically launching Python emulator in the background...");
        // This command assumes the executable is run from a 'build' directory
        // located at the project root. The '&' makes it run in the background on Linux.
        const char* command = "xterm -T \"Robot Emulator\" -hold -e \"python3  /home/rdt/Desktop/RDT/RobotControl_MVP/motion_interface_hal/robot_utility.py --emulator\" &";
        int result = std::system(command);
        if (result != 0) {
            LOG_ERROR("TestMain", "Failed to launch emulator script. Please run it manually.");
            // We can choose to continue or exit. Let's try to continue.
        }
        LOG_INFO("TestMain", "Waiting 1 second for emulator to initialize...");
        std::this_thread::sleep_for(1s); // Give the script time to start and bind sockets.
    } else {
        LOG_INFO("TestMain", "Manual mode: Ensure python/robot_utility.py --emulator is running.");
    }

    // --- 1. CONFIGURATION ---
    InterfaceConfig config;
    config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
    config.udp_control_config.remote_ip = "127.0.0.1";
    config.udp_control_config.remote_port = 50001;
    config.udp_control_config.local_port = 50002;
    config.simulation_initial_joints.fromAngleArray({0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad});
    config.debug_stream_config.enabled = true;
    config.debug_stream_config.stream_frequency_hz = 5;

    LOG_INFO("TestMain", "Configuration created. Realtime interface: UDP, Debug stream: ENABLED.");

    // --- 2. INITIALIZATION ---
    MasterHardwareInterface master_iface(config);
    if (!master_iface.connect()) {
        LOG_CRITICAL("TestMain", "Failed to connect MasterHardwareInterface. Aborting.");
        return 1;
    }
    LOG_INFO("TestMain", "Master interface connected. All background threads are running.");
    std::this_thread::sleep_for(200ms);

    // --- 3. SIMULATION MODE TEST ---
    LOG_INFO("TestMain", "\n--- PHASE 1: Operating in SIMULATION mode (default) ---");
    LOG_INFO_F("TestMain", "Current Mode: %s", master_iface.getCurrentMode() == MasterHardwareInterface::ActiveMode::Simulation ? "Simulation" : "Realtime");
    printAxisSet("Initial Sim Feedback", master_iface.getLatestFeedback());
    AxisSet sim_command;
    sim_command.fromAngleArray({(15.0_deg).toRadians(), (-10.0_deg).toRadians(), (5.0_deg).toRadians(), 0.0_rad, 0.0_rad, 0.0_rad});
    master_iface.sendCommand(sim_command);
    std::this_thread::sleep_for(50ms);
    printAxisSet("Updated Sim Feedback", master_iface.getLatestFeedback());

    // --- 4. SYNC CHECK AND SWITCH (FAIL) ---
    LOG_INFO("TestMain", "\n--- PHASE 2: Attempting to switch to REALTIME (expected to fail) ---");
    MasterHardwareInterface::SyncInfo sync_info = master_iface.checkSync(1.0_deg);
    LOG_INFO_F("TestMain", "Sync check: In Sync = %s, Max Deviation = %.2f deg", sync_info.is_in_sync ? "YES" : "NO", sync_info.max_deviation_deg);
    printAxisSet("Current Sim State ", sync_info.simulated_joints);
    printAxisSet("Current Real State (UDP)", sync_info.real_joints);
    auto switch_result = master_iface.switchTo(MasterHardwareInterface::ActiveMode::Realtime);
    if (switch_result == MasterHardwareInterface::SwitchResult::NotInSync) {
        LOG_INFO("TestMain", "Switch failed due to lack of sync. This is CORRECT.");
    } else {
        LOG_ERROR("TestMain", "Switch test FAILED. It should not have succeeded.");
    }

    // --- 5. FORCE SYNC AND SWITCH (SUCCESS) ---
    LOG_INFO("TestMain", "\n--- PHASE 3: Forcing sync and switching to REALTIME ---");
    master_iface.forceSyncSimulationToReal();
    LOG_INFO("TestMain", "Forced sync. Waiting for states to align...");
    std::this_thread::sleep_for(100ms);
    sync_info = master_iface.checkSync(1.0_deg);
    LOG_INFO_F("TestMain", "Sync check after force sync: In Sync = %s, Max Deviation = %.2f deg", sync_info.is_in_sync ? "YES" : "NO", sync_info.max_deviation_deg);
    printAxisSet("Synced Sim State ", sync_info.simulated_joints);
    printAxisSet("Current Real State (UDP)", sync_info.real_joints);
    switch_result = master_iface.switchTo(MasterHardwareInterface::ActiveMode::Realtime);
    if (switch_result == MasterHardwareInterface::SwitchResult::Success) {
        LOG_INFO("TestMain", "Switch to Realtime SUCCEEDED. This is CORRECT.");
    } else {
        LOG_ERROR("TestMain", "Switch test FAILED. It should have succeeded this time.");
    }

    // --- 6. REALTIME (UDP) MODE TEST ---
    LOG_INFO("TestMain", "\n--- PHASE 4: Operating in REALTIME (UDP) mode ---");
    LOG_INFO_F("TestMain", "Current Mode: %s", master_iface.getCurrentMode() == MasterHardwareInterface::ActiveMode::Simulation ? "Simulation" : "Realtime");
    printAxisSet("Initial Real Feedback", master_iface.getLatestFeedback());
    AxisSet real_command;
    real_command.fromAngleArray({(12.0_deg).toRadians(), (22.0_deg).toRadians(), (32.0_deg).toRadians(), 0.0_rad, 0.0_rad, 0.0_rad});
    master_iface.sendCommand(real_command);
    LOG_INFO("TestMain", "Sent command to UDP. Waiting for feedback loop to update...");
    std::this_thread::sleep_for(100ms);
    printAxisSet("Updated Real Feedback", master_iface.getLatestFeedback());
    
    // --- 7. SWITCH BACK TO SIM ---
    LOG_INFO("TestMain", "\n--- PHASE 5: Switching back to SIMULATION mode ---");
    master_iface.switchTo(MasterHardwareInterface::ActiveMode::Simulation);
    LOG_INFO_F("TestMain", "Current Mode: %s", master_iface.getCurrentMode() == MasterHardwareInterface::ActiveMode::Simulation ? "Simulation" : "Realtime");

    // --- 8. SHUTDOWN ---
    LOG_INFO("TestMain", "\n--- Test finished. Disconnecting... (Debug stream will run for 3 more seconds) ---");
    LOG_INFO("TestMain", "You can run 'python3 ../python/robot_utility.py --debug-listener' to see the output.");
    std::this_thread::sleep_for(3s);
    master_iface.disconnect();
    LOG_INFO("TestMain", "Disconnected. Application will now exit.");
    
    // Attempt to clean up the background process if we started it
    if (LAUNCH_EMULATOR_AUTOMATICALLY) {
        LOG_INFO("TestMain", "Attempting to clean up background emulator process...");
        // This is a simple, somewhat crude way to clean up. A more robust solution
        // would involve process IDs (pids).
        std::system("pkill -f 'robot_utility.py --emulator'");
    }

    return 0;
}