// main.cpp
#include <iostream>
#include <string>
#include <vector>
#include <atomic>
#include <csignal>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <memory>

#include "EthercatInterface.h"
#include "Logger.h"
#include "RobotConfig.h"
#include "DataTypes.h"
#include "CiA402.h"
#include "AxisConfiguration.h"

using namespace RDT;
using namespace std::chrono_literals;

// --- Глобальные переменные и константы ---
static std::atomic<bool> g_running(true);
static AxisSet g_target_positions{};
const std::string ETHERCAT_IFACE_NAME = "enp1s0";
const unsigned int COMM_CYCLE_MS = 4;
const std::string CALIBRATION_FILE = "calibration.conf";

void handle_signal(int) {
    g_running.store(false);
    std::cout << "\nCtrl+C detected. Shutting down..." << std::endl;
}

void communication_thread_func(EthercatInterface* ecat_iface) {
    while (g_running.load()) {
        if (ecat_iface && ecat_iface->isConnected()) {
            ecat_iface->sendCommand(g_target_positions);
        }
        // Задержка будет внутри sendCommand, здесь можно ее убрать для чистоты
        // Но для снижения нагрузки на CPU в случае проблем с sendCommand, оставим.
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void monitor_thread_func(EthercatInterface* ecat_iface) {
    const int slave_count = ecat_iface->getConnectedSlavesCount();
    std::vector<DriveState> last_states(slave_count, DriveState::NotReadyToSwitchOn);

    while (g_running.load()) {
        std::ostringstream status_line_stream;
        AxisSet feedback = ecat_iface->getLatestFeedback();

        for (int i = 0; i < slave_count; ++i) {
            DriveState current_state = ecat_iface->getAxisDriveState(i);
            std::string state_str;
            switch(current_state) {
                case DriveState::OperationEnabled: state_str = "OPERATING"; break;
                case DriveState::Fault:            state_str = "FAULT"; break;
                case DriveState::SwitchOnDisabled: state_str = "DISABLED"; break;
                case DriveState::SwitchedOn:       state_str = "SWITCHED ON"; break;
                case DriveState::ReadyToSwitchOn:  state_str = "READY"; break;
                default:                           state_str = "TRANSITION"; break;
            }
            
            status_line_stream << " | A" << i << ": " 
                               << (ecat_iface->isAxisEnableRequested(i) ? "[EN]" : "[--]")
                               << std::left << std::setw(12) << state_str
                               << std::fixed << std::setprecision(2) << std::right << std::setw(8)
                               << feedback.at(i).angle.toDegrees().value() << " deg";
        }
        status_line_stream << " |";

        bool has_event = false;
        for (int i = 0; i < slave_count; ++i) {
            DriveState current_state = ecat_iface->getAxisDriveState(i);
            if (current_state != last_states[i]) {
                if (current_state == DriveState::Fault) {
                    std::cout << "\n[EVENT] Axis " << i << " entered FAULT state. Error: " << ecat_iface->getAxisErrorString(i) << std::endl;
                } else {
                    std::cout << "\n[EVENT] Axis " << i << " state changed." << std::endl;
                }
                last_states[i] = current_state;
                has_event = true;
            }
        }
        
        std::cout << status_line_stream.str() << "    \r" << std::flush;
        if (has_event) {
            std::cout << "\nEnter command > " << std::flush;
        }
        
        std::this_thread::sleep_for(250ms);
    }
}

int main(int, char**) {
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);
    
    Logger::setLogLevel(LogLevel::Info);
    LOG_INFO("EcatMain", "--- EtherCAT Console ---");

    InterfaceConfig iface_config;
    iface_config.ethercat_config.iface_name = ETHERCAT_IFACE_NAME;
    iface_config.motion_manager_cycle_ms = COMM_CYCLE_MS;
    auto ecat_iface = std::make_unique<EthercatInterface>(iface_config);

    if (!ecat_iface->connect()) {
        LOG_CRITICAL("EcatMain", "Failed to connect to EtherCAT master.");
        return 1;
    }

    // Инициализируем целевые позиции текущими, чтобы избежать скачка
    AxisSet initial_feedback = ecat_iface->getLatestFeedback();
    if (ecat_iface->loadCalibration(CALIBRATION_FILE)) {
        LOG_INFO("Main", "Calibration loaded successfully.");
        // После загрузки калибровки снова получаем фидбек, чтобы целевые были корректными
        initial_feedback = ecat_iface->getLatestFeedback();
    }
    g_target_positions = initial_feedback;

    AxisConfiguration axis_config;
    for (int i = 0; i < ecat_iface->getConnectedSlavesCount(); ++i) {
        ecat_iface->configureAxis(i, axis_config);
    }

    std::jthread comm_thread(communication_thread_func, ecat_iface.get());
    std::jthread mon_thread(monitor_thread_func, ecat_iface.get());

    std::cout << "\nAvailable commands:\n"
              << "  e <axis>      - Enable axis (e.g., 'e 0')\n"
              << "  d <axis>      - Disable axis (e.g., 'd 1')\n"
              << "  r <axis>      - Reset fault on axis\n"
              << "  c <axis>      - Calibrate current position as zero\n"
              << "  s             - Save current calibrations to file\n"
              << "  m <axis> <deg> - Move axis to absolute angle (e.g., 'm 0 90.5')\n"
              << "  exit          - Quit the application\n" << std::endl;

    std::string line;
    while (g_running.load()) {
        std::cout << "Enter command > " << std::flush;
        
        if (!std::getline(std::cin, line) || line == "exit") {
            g_running.store(false);
            break;
        }
        if (!g_running.load()) break;

        std::stringstream ss(line);
        std::string cmd;
        ss >> cmd;
        int idx = -1;

        if (cmd == "s") {
            if (ecat_iface->saveCalibration(CALIBRATION_FILE)) {
                 std::cout << "\n[CMD] Position calibration saved to " << CALIBRATION_FILE << std::endl;
            }
        } else if (cmd == "m") {
            double angle_deg = 0.0;
            if (!(ss >> idx) || idx < 0 || idx >= ecat_iface->getConnectedSlavesCount()) {
                std::cout << "\n[ERROR] Invalid or missing axis number for move command." << std::endl; continue;
            }
            if (!(ss >> angle_deg)) {
                 std::cout << "\n[ERROR] Invalid or missing angle for move command." << std::endl; continue;
            }
            g_target_positions.at(idx).angle = Degrees(angle_deg).toRadians();
            std::cout << "\n[CMD] Set target for axis " << idx << " to " << angle_deg << " deg." << std::endl;

        } else if (cmd == "e" || cmd == "d" || cmd == "r" || cmd == "c") {
            if (!(ss >> idx) || idx < 0 || idx >= ecat_iface->getConnectedSlavesCount()) {
                std::cout << "\n[ERROR] Invalid or missing axis number." << std::endl; continue;
            }
            if (cmd == "e") ecat_iface->enableAxis(idx);
            else if (cmd == "d") ecat_iface->disableAxis(idx);
            else if (cmd == "r") ecat_iface->resetAxisFault(idx);
            else if (cmd == "c") ecat_iface->calibrateAxisZero(idx);
        } else if (!cmd.empty()) {
            std::cout << "\n[ERROR] Unknown command: '" << cmd << "'." << std::endl;
        }
    }

    LOG_INFO("EcatMain", "Shutdown signal received. Disconnecting...");
    ecat_iface->disconnect();
    LOG_INFO("EcatMain", "Application finished.");

    return 0;
}