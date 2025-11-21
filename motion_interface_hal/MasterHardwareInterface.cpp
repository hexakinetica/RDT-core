// MasterHardwareInterface.cpp
#include "MasterHardwareInterface.h"
#include "InternalSimulation.h"
#include "EthercatInterface.h"
#include "UdpInterface.h"
#include "Logger.h"

namespace RDT {

using namespace std::chrono_literals;

MasterHardwareInterface::MasterHardwareInterface(const InterfaceConfig& config) : config_(config) {
    sim_interface_ = std::make_unique<InternalSimulation>(config);

    if (config_.realtime_type == InterfaceConfig::RealtimeInterfaceType::EtherCAT) {
        real_interface_ = std::make_unique<EthercatInterface>(config);
        LOG_INFO(MODULE_NAME, "Realtime interface selected: EtherCAT");
    } else if (config_.realtime_type == InterfaceConfig::RealtimeInterfaceType::Udp) {
        real_interface_ = std::make_unique<UdpInterface>(config);
        LOG_INFO(MODULE_NAME, "Realtime interface selected: UDP");
    } else {
        LOG_INFO(MODULE_NAME, "No realtime interface selected. Running in simulation-only mode.");
    }

    if (config_.debug_stream_config.enabled) {
        NetworkConfig debug_net_conf;
        debug_net_conf.remote_ip = config_.debug_stream_config.destination_ip;
        debug_net_conf.remote_port = config_.debug_stream_config.destination_port;
        debug_net_conf.local_port = 0;
        debug_udp_peer_ = std::make_unique<UdpPeer>(debug_net_conf);
    }
}

MasterHardwareInterface::~MasterHardwareInterface() {
    disconnect();
}

bool MasterHardwareInterface::connect() {
    bool sim_ok = sim_interface_->connect();
    bool real_ok = true; 
    if (real_interface_) {
        real_ok = real_interface_->connect();
    }
    
    if (sim_ok) {
        if (debug_udp_peer_) {
            debug_stream_thread_ = std::jthread([this](std::stop_token stoken){
                this->debugStreamThreadLoop(stoken);
            });
        }
        return true;
    }
    LOG_ERROR(MODULE_NAME, "Failed to connect the simulation sub-interface.");
    return false;
}

void MasterHardwareInterface::disconnect() {
    if (debug_stream_thread_.joinable()) {
        debug_stream_thread_.request_stop();
        debug_stream_thread_.join();
    }
    if (sim_interface_) sim_interface_->disconnect();
    if (real_interface_) real_interface_->disconnect();
}

bool MasterHardwareInterface::isConnected() const {
    if (active_mode_.load() == ActiveMode::Simulation) {
        return sim_interface_ && sim_interface_->isConnected();
    } else {
        return real_interface_ && real_interface_->isConnected();
    }
}

CommStatus MasterHardwareInterface::sendCommand(const AxisSet& command) {
    if (active_mode_.load() == ActiveMode::Simulation) {
        if (sim_interface_) {
            return sim_interface_->sendCommand(command);
        }
        return CommStatus::NotConnected;
    } else { // Realtime mode
        if (real_interface_) {
            return real_interface_->sendCommand(command);
        }
        return CommStatus::NotConnected;
    }
}

AxisSet MasterHardwareInterface::getLatestFeedback() const {
    if (active_mode_.load() == ActiveMode::Simulation) {
        return sim_interface_->getLatestFeedback();
    } else {
        if (real_interface_) {
            return real_interface_->getLatestFeedback();
        }
        return AxisSet{};
    }
}

void MasterHardwareInterface::setState(const AxisSet& state) {
    if (sim_interface_) {
        sim_interface_->setState(state);
    }
}

MasterHardwareInterface::SyncInfo MasterHardwareInterface::checkSync(Degrees tolerance) {
    SyncInfo info;
    info.is_in_sync = false;
    info.max_deviation_deg = 999.0;

    if (!real_interface_ || !real_interface_->isConnected()) {
        info.simulated_joints = sim_interface_ ? sim_interface_->getLatestFeedback() : AxisSet{};
        info.real_joints = AxisSet{};
        return info;
    }
    
    info.simulated_joints = sim_interface_->getLatestFeedback();
    info.real_joints = real_interface_->getLatestFeedback();
    
    info.max_deviation_deg = 0.0;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        double diff = (info.simulated_joints.at(i).angle - info.real_joints.at(i).angle).abs().toDegrees().value();
        if (diff > info.max_deviation_deg) {
            info.max_deviation_deg = diff;
        }
    }
    
    info.is_in_sync = (info.max_deviation_deg <= tolerance.value());
    return info;
}

void MasterHardwareInterface::forceSyncSimulationToReal() {
    if (real_interface_ && real_interface_->isConnected()) {
        setState(real_interface_->getLatestFeedback());
    }
}

MasterHardwareInterface::SwitchResult MasterHardwareInterface::switchTo(ActiveMode mode) {
    // ========================================================================
    // --- NEW LOGIC for Real -> Sim synchronization ---
    // ========================================================================
    if (getCurrentMode() == ActiveMode::Realtime && mode == ActiveMode::Simulation) {
        LOG_INFO(MODULE_NAME, "Switching Real->Sim: Syncing simulation to last known real hardware state.");
        if (real_interface_ && sim_interface_ && real_interface_->isConnected()) {
            AxisSet last_real_feedback = real_interface_->getLatestFeedback();
            sim_interface_->setState(last_real_feedback);
            LOG_INFO(MODULE_NAME, "Simulation state updated to match real hardware.");
        } else {
            LOG_WARN(MODULE_NAME, "Could not sync Real->Sim: Real interface not available or not connected.");
        }
    }
    // ========================================================================

    // --- Existing logic for Sim -> Real switch ---
    if (mode == ActiveMode::Realtime) {
        if (!real_interface_ || !real_interface_->isConnected()) {
            LOG_ERROR(MODULE_NAME, "Cannot switch to Realtime: real interface is not available or not connected.");
            return SwitchResult::RealtimeNotAvailable;
        }
        if (!checkSync(0.5_deg).is_in_sync) {
            LOG_WARN(MODULE_NAME, "Cannot switch to Realtime: simulation and real states are not in sync.");
            return SwitchResult::NotInSync;
        }
    }
    
    // --- Common logic for all successful switches ---
    active_mode_.store(mode);
    LOG_INFO_F(MODULE_NAME, "Active mode switched to %s.", (mode == ActiveMode::Simulation ? "Simulation" : "Realtime"));
    return SwitchResult::Success;
}

MasterHardwareInterface::ActiveMode MasterHardwareInterface::getCurrentMode() const {
    return active_mode_.load();
}

bool MasterHardwareInterface::isRealInterfaceConnected() const {
    return real_interface_ && real_interface_->isConnected();
}

void MasterHardwareInterface::debugStreamThreadLoop(std::stop_token stoken) {
    if (!debug_udp_peer_ || !config_.debug_stream_config.enabled) return;
    if (debug_udp_peer_->connect() != 0) {
        LOG_ERROR(MODULE_NAME, "Failed to initialize UDP peer for debug stream.");
        return;
    }

    // --- ИСПРАВЛЕНИЕ 1: Проверка на ноль и использование float-деления ---
    if (config_.debug_stream_config.stream_frequency_hz <= 0) {
        LOG_ERROR(MODULE_NAME, "Debug stream frequency must be positive. Stream stopped.");
        return;
    }
    
    // Используем 1000.0 для деления с плавающей точкой, чтобы получить точный период
    auto period = std::chrono::duration<double>(1.0 / static_cast<double>(config_.debug_stream_config.stream_frequency_hz));

    
    auto next_cycle_time = std::chrono::steady_clock::now();
    auto start_time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

    LOG_INFO_F(MODULE_NAME, "Debug UDP stream started. Target: %s:%d, Frequency: %d Hz",
        config_.debug_stream_config.destination_ip.c_str(),
        config_.debug_stream_config.destination_port,
        config_.debug_stream_config.stream_frequency_hz);

    while (!stoken.stop_requested()) {
        // --- ИСПРАВЛЕНИЕ 2: Использование 'sleep_until' для стабильной частоты ---
        auto period_ns = std::chrono::duration_cast<std::chrono::steady_clock::duration>(period);
        next_cycle_time += period_ns;

        // Получаем таймштамп в самом начале цикла
        auto now = std::chrono::system_clock::now();
        auto timestamp_ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

        // Получаем все три состояния 
        AxisSet actual_fb = getLatestFeedback();
        AxisSet sim_fb = sim_interface_ ? sim_interface_->getLatestFeedback() : AxisSet{};
        AxisSet real_fb = (real_interface_ && real_interface_->isConnected()) ? real_interface_->getLatestFeedback() : AxisSet{};
        
        std::ostringstream ss;
        ss << std::fixed << std::setprecision(3);

        // 1. Формируем строку для ACTUAL с таймштампом
        ss << "[ TIME: " << (timestamp_ms-start_time_ms)/1000.0 << "; ";
        ss << "ACTUAL: ";
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            ss << "A" << (i + 1) << ":" << actual_fb.at(i).angle.toDegrees().value() << (i == ROBOT_AXES_COUNT - 1 ? "" : ",");
        }
        
        // 2. Формируем строку для REAL
        ss << "; REAL: ";
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            ss << "A" << (i + 1) << ":" << real_fb.at(i).angle.toDegrees().value() << (i == ROBOT_AXES_COUNT - 1 ? "" : ",");
        }

        // 3. Формируем строку для SIM
        ss << "; SIM: ";
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            ss << "A" << (i + 1) << ":" << sim_fb.at(i).angle.toDegrees().value() << (i == ROBOT_AXES_COUNT - 1 ? "" : ",");
        }
         ss << " ]\n";
        std::string payload = ss.str();
        std::vector<char> data(payload.begin(), payload.end());
        if (debug_udp_peer_->send(data) < 0) {
            LOG_DEBUG(MODULE_NAME, "Failed to send debug UDP packet.");
        }

        // Ожидаем до начала следующего цикла
        std::this_thread::sleep_until(next_cycle_time);
    }
    LOG_INFO(MODULE_NAME, "Debug UDP stream stopped.");
}

} // namespace RDT