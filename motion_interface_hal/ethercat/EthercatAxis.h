// EthercatAxis.h
#ifndef ETHERCAT_AXIS_H
#define ETHERCAT_AXIS_H

#pragma once
#include "DataTypes.h"
#include "Units.h"
#include "CiA402.h"
#include "AxisConfiguration.h"
#include <ecrt.h>
#include <string>
#include <atomic>

namespace RDT {

// Перечисление для состояний конечного автомата CiA 402
enum class DriveState {
    NotReadyToSwitchOn, SwitchOnDisabled, ReadyToSwitchOn, SwitchedOn,
    OperationEnabled, QuickStopActive, FaultReactionActive, Fault
};

class EthercatAxis {
public:
    EthercatAxis(uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code);
    ~EthercatAxis() = default;

    EthercatAxis(const EthercatAxis&) = delete;
    EthercatAxis& operator=(const EthercatAxis&) = delete;

    // Методы для фазы конфигурации
    void registerPdosInDomain(ec_domain_t* domain);
    void configure(ec_slave_config_t* sc, const AxisConfiguration& config);
    
    // Методы для RT-цикла
    void processFeedback(const uint8_t* domain_pd);
    void writeCommand(uint8_t* domain_pd, Radians target_position);

    // Методы для NRT-взаимодействия
    void requestFaultReset();
    void requestZeroCalibration();
    void requestEnable();
    void requestDisable();
    void setZeroOffset(int32_t offset_counts);

    // Геттеры состояния
    [[nodiscard]] Axis getActualState() const;
    [[nodiscard]] bool isEnabled() const;
    [[nodiscard]] bool isFaulted() const;
    [[nodiscard]] uint16_t getErrorCode() const;
    [[nodiscard]] std::string getErrorString() const;
    [[nodiscard]] DriveState getDriveState() const;
    [[nodiscard]] uint16_t getStatusword() const;
    [[nodiscard]] int32_t getZeroOffset() const;
    [[nodiscard]] int32_t getRawEncoderValue() const;
    [[nodiscard]] bool isEnableRequested() const;

private:
    void updateDriveState(uint16_t statusword);
    
    // Идентификационные данные
    const uint16_t alias_;
    const uint16_t position_;
    const uint32_t vendor_id_;
    const uint32_t product_code_;

    // Конфигурация и калибровка
    AxisConfiguration config_;
    double counts_per_radian_ = 1.0;
    int32_t zero_offset_counts_ = 0;

    // Смещения PDO в домене
    unsigned int offset_controlword_, offset_target_pos_, offset_modes_of_op_;
    unsigned int offset_statusword_, offset_actual_pos_, offset_error_code_, offset_actual_torque_;
    unsigned int offset_max_velocity_;

    // Текущее состояние
    DriveState current_state_ = DriveState::NotReadyToSwitchOn;
    uint16_t last_error_code_ = 0;
    uint16_t last_statusword_ = 0;
    int32_t raw_encoder_value_ = 0;
    Axis actual_axis_state_;
    
    // Флаги-запросы от NRT-потока к RT-потоку
    std::atomic<bool> fault_reset_requested_ = false;
    std::atomic<bool> zero_calibration_requested_ = false;
    std::atomic<bool> operation_enabled_request_ = false; // <-- НОВЫЙ ФЛАГ
};

} // namespace RDT

#endif // ETHERCAT_AXIS_H