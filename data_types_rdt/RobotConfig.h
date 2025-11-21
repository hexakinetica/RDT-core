// RobotConfig.h
#ifndef ROBOT_CONFIG_H
#define ROBOT_CONFIG_H

#pragma once

#include "DataTypes.h"
#include <string>
#include <memory>

namespace RDT {

using namespace RDT::literals;

/**
 * @struct ControllerConfig
 * @brief Configuration parameters for the main RobotController and its components.
 */
struct ControllerConfig {
    /// @brief Period for the non-real-time control loop in RobotController.
    std::chrono::milliseconds control_loop_period = std::chrono::milliseconds(50);
    /// @brief How often the RobotController processes the batch of feedback from MotionManager.
    std::chrono::milliseconds feedback_processing_interval = std::chrono::milliseconds(100);
    /// @brief Time step for the TrajectoryPlanner's internal interpolation.
    Seconds planner_dt_sample = Seconds(0.02);
    /// @brief How far ahead the TrajectoryPlanner generates points in one go.
    Seconds planner_window_duration = Seconds(0.2);
    /// @brief Period for the real-time loop in MotionManager.
    unsigned int motion_manager_cycle_ms = 20;
};

/**
 * @struct UdpConfig
 * @brief Network configuration for a UDP-based interface.
 */
struct UdpConfig {
    std::string remote_ip = "127.0.0.1"; ///< IP address of the remote peer (the robot).
    uint16_t remote_port = 50001;        ///< Port on the remote peer to send commands to.
    uint16_t local_port = 50002;         ///< Local port to bind to for receiving feedback.
    int timeout_ms = 100;                ///< Receive timeout in milliseconds.
};

/**
 * @struct EthercatConfig
 * @brief Configuration for an EtherCAT-based interface.
 */
// НОВАЯ СТРУКТУРА для конфигурации SDO
struct SdoConfig {
    uint16_t index;
    uint8_t subindex;
    uint32_t value;
    size_t size = 4; // Размер данных в байтах (по умолчанию 4 для UDINT/DINT)
};

/**
 * @struct EthercatConfig
 * @brief Configuration for an EtherCAT-based interface.
 */
struct EthercatConfig {
    std::string iface_name = "enp1s0"; ///< Network interface name for EtherCAT master.
    // НОВОЕ ПОЛЕ для хранения конфигураций SDO
    std::vector<SdoConfig> sdo_configs;
};


/**
 * @struct UdpDebugStreamConfig
 * @brief Configuration for the optional UDP debug stream.
 */
struct UdpDebugStreamConfig {
    bool enabled = true;
    std::string destination_ip = "192.168.70.102";
    uint16_t destination_port = 60000;
    int stream_frequency_hz = 50;
};

/**
 * @struct InterfaceConfig
 * @brief Top-level configuration for all hardware and simulation interfaces.
 */
struct InterfaceConfig {
    /**
     * @enum RealtimeInterfaceType
     * @brief Defines the type of real hardware interface to use.
     */
    enum class RealtimeInterfaceType {
        None,     ///< Use simulation only.
        EtherCAT, ///< Use the EtherCAT interface.
        Udp       ///< Use the UDP interface.
    };
    
    RealtimeInterfaceType realtime_type = RealtimeInterfaceType::None;
    /// @brief Period for the real-time loop in MotionManager AND the hardware interface.
    unsigned int motion_manager_cycle_ms = 4; // 4 ms = 250 Hz по умолчанию

    UdpConfig udp_control_config;
    EthercatConfig ethercat_config;
    UdpDebugStreamConfig debug_stream_config;
    AxisSet simulation_initial_joints;
};

} // namespace RDT
#endif // ROBOT_CONFIG_H