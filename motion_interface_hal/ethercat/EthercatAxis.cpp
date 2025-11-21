// EthercatAxis.cpp
#include "EthercatAxis.h"
#include "Logger.h"
#include <cmath>
#include <atomic>

namespace RDT {

EthercatAxis::EthercatAxis(uint16_t alias, uint16_t position, uint32_t vendor_id, uint32_t product_code)
    : alias_(alias), position_(position), vendor_id_(vendor_id), product_code_(product_code)
{
    actual_axis_state_ = {}; // Инициализация нулями
}

void EthercatAxis::registerPdosInDomain(ec_domain_t* domain) {
    ec_pdo_entry_reg_t domain_regs[] = {
        // RxPDO
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::Controlword, 0, &offset_controlword_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::TargetPosition, 0, &offset_target_pos_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::ModesOfOperation, 0, &offset_modes_of_op_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::MaxProfileVelocity, 0, &offset_max_velocity_}, 
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::Statusword, 0, &offset_statusword_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::ActualPosition, 0, &offset_actual_pos_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::ErrorCode, 0, &offset_error_code_},
        {alias_, position_, vendor_id_, product_code_, CiA402::Objects::ActualTorque, 0, &offset_actual_torque_},
        {}
    };


    if (ecrt_domain_reg_pdo_entry_list(domain, domain_regs)) {
        LOG_ERROR_F("EthercatAxis", "PDO entry registration failed for slave at position %u.", position_);
    }
}

void EthercatAxis::configure(ec_slave_config_t* sc, const AxisConfiguration& config) {
    config_ = config;
    
    counts_per_radian_ = (static_cast<double>(1 << config_.encoder_resolution) * config_.gear_ratio) / (2.0 * UnitConstants::PI);
    //std::cout<<counts_per_radian_<<std::endl;

    // Конвертируем градусы в радианы, а затем в тики энкодера
    ecrt_slave_config_sdo32(sc, CiA402::Objects::SoftwarePositionLimit, 1, 
        static_cast<int32_t>(config_.software_limit_min.toRadians().value() * counts_per_radian_));
    ecrt_slave_config_sdo32(sc, CiA402::Objects::SoftwarePositionLimit, 2, 
        static_cast<int32_t>(config_.software_limit_max.toRadians().value() * counts_per_radian_));
    
    // Конвертируем динамические параметры из градусов/с... в единицы привода (тики/с...)
    //ecrt_slave_config_sdo32(sc, CiA402::Objects::MaxProfileVelocity, 0, 
     //   static_cast<uint32_t>(0));

        
    //ecrt_slave_config_sdo32(sc, CiA402::Objects::ProfileAcceleration, 0, 
    //    static_cast<uint32_t>(config_.profile_acceleration.toRadiansPerSecondSq().value() * counts_per_radian_));
    //ecrt_slave_config_sdo32(sc, CiA402::Objects::ProfileDeceleration, 0, 
    //    static_cast<uint32_t>(config_.profile_deceleration.toRadiansPerSecondSq().value() * counts_per_radian_));
    
   // ecrt_slave_config_sdo32(sc, CiA402::Objects::FollowingErrorWindow, 0, config_.following_error_window.toRadians().value() * counts_per_radian_);
    
    ecrt_slave_config_sdo16(sc, CiA402::Objects::PositiveTorqueLimit, 0, config_.torque_limit);
    ecrt_slave_config_sdo16(sc, CiA402::Objects::NegativeTorqueLimit, 0, config_.torque_limit);
}

void EthercatAxis::processFeedback(const uint8_t* domain_pd) {
    last_statusword_ = EC_READ_U16(domain_pd + offset_statusword_);
    raw_encoder_value_ = EC_READ_S32(domain_pd + offset_actual_pos_);
    
    updateDriveState(last_statusword_);

    if (current_state_ == DriveState::Fault) {
        last_error_code_ = EC_READ_U16(domain_pd + offset_error_code_);
    } else {
        last_error_code_ = 0;
    }

    if (zero_calibration_requested_.load()) {
        zero_offset_counts_ = raw_encoder_value_;
        zero_calibration_requested_ = false;
        LOG_INFO_F("EthercatAxis", "Axis %u: Zero offset calibrated to %d", position_, zero_offset_counts_);
    }

    actual_axis_state_.angle = Radians(static_cast<double>(raw_encoder_value_ - zero_offset_counts_) / counts_per_radian_);
    actual_axis_state_.torque = static_cast<double>(EC_READ_S16(domain_pd + offset_actual_torque_));
    actual_axis_state_.velocity = RadiansPerSecond(0);
    actual_axis_state_.servo_enabled = (current_state_ == DriveState::OperationEnabled);
}

void EthercatAxis::writeCommand(uint8_t* domain_pd, Radians target_position) {
    uint16_t controlword = 0;

    if (fault_reset_requested_.load()) {
        if (current_state_ == DriveState::Fault) {
            controlword = CiA402::Controlword::FaultReset;
        }
        fault_reset_requested_ = false; 
    } else {
        // Если мы хотим, чтобы привод был активен
        if (operation_enabled_request_.load()) {
            switch (current_state_) {
                case DriveState::Fault:
                    break;
                case DriveState::SwitchOnDisabled:
                    controlword = CiA402::Controlword::CmdShutdown;
                    break;
                case DriveState::ReadyToSwitchOn:
                    controlword = CiA402::Controlword::CmdSwitchOn;
                    break;
                case DriveState::SwitchedOn:
                    controlword = CiA402::Controlword::CmdEnable;
                    EC_WRITE_S8(domain_pd + offset_modes_of_op_, CiA402::OperationMode::CyclicSynchronousPosition);
                    break;
                case DriveState::OperationEnabled:
                    controlword = CiA402::Controlword::CmdEnable;
                    break;
                default:
                    controlword = CiA402::Controlword::CmdShutdown;
                    break;
            }
        // Если мы хотим, чтобы привод был НЕактивен
        } else {
            switch (current_state_) {
                // Если уже в рабочем состоянии, выключаем его командой Shutdown
                case DriveState::OperationEnabled:
                case DriveState::SwitchedOn:
                case DriveState::ReadyToSwitchOn:
                    controlword = CiA402::Controlword::CmdShutdown;
                    break;
                // В остальных состояниях ничего не делаем, он и так неактивен
                default:
                    break;
            }
        }
    }

    EC_WRITE_U16(domain_pd + offset_controlword_, controlword);

    // Отправляем целевую позицию только если привод включен.
    // В противном случае, отправляем текущую, чтобы избежать скачка при включении.
    if (current_state_ == DriveState::OperationEnabled) {
        int32_t target_counts = static_cast<int32_t>(target_position.value() * counts_per_radian_) + zero_offset_counts_;
        EC_WRITE_S32(domain_pd + offset_target_pos_, target_counts);
    } else {
        EC_WRITE_S32(domain_pd + offset_target_pos_, raw_encoder_value_);
    }
    
    uint32_t max_vel_inc_s = static_cast<uint32_t>(config_.max_velocity.toRadiansPerSecond().value() * counts_per_radian_);
    EC_WRITE_U32(domain_pd + offset_max_velocity_, max_vel_inc_s);
}

void EthercatAxis::requestFaultReset() { fault_reset_requested_ = true; }
void EthercatAxis::requestZeroCalibration() { zero_calibration_requested_ = true; }
void EthercatAxis::setZeroOffset(int32_t offset_counts) { zero_offset_counts_ = offset_counts; }
void EthercatAxis::requestEnable() { operation_enabled_request_ = true; }
void EthercatAxis::requestDisable() { operation_enabled_request_ = false; }
bool EthercatAxis::isEnableRequested() const { return operation_enabled_request_.load(); }

Axis EthercatAxis::getActualState() const { return actual_axis_state_; }
int32_t EthercatAxis::getZeroOffset() const { return zero_offset_counts_; }
int32_t EthercatAxis::getRawEncoderValue() const { return raw_encoder_value_; }
bool EthercatAxis::isEnabled() const { return current_state_ == DriveState::OperationEnabled; }
bool EthercatAxis::isFaulted() const { return current_state_ == DriveState::Fault; }
uint16_t EthercatAxis::getErrorCode() const { return last_error_code_; }
std::string EthercatAxis::getErrorString() const { return CiA402::getErrorString(last_error_code_); }
DriveState EthercatAxis::getDriveState() const { return current_state_; }
uint16_t EthercatAxis::getStatusword() const { return last_statusword_; }

void EthercatAxis::updateDriveState(uint16_t statusword) {
    if ((statusword & CiA402::Statusword::MaskFault) == CiA402::Statusword::ValFault) {
        current_state_ = DriveState::Fault;
    } else if ((statusword & CiA402::Statusword::MaskOperationEnabled) == CiA402::Statusword::ValOperationEnabled) {
        current_state_ = DriveState::OperationEnabled;
    } else if ((statusword & CiA402::Statusword::MaskSwitchedOn) == CiA402::Statusword::ValSwitchedOn) {
        current_state_ = DriveState::SwitchedOn;
    } else if ((statusword & CiA402::Statusword::MaskReadyToSwitchOn) == CiA402::Statusword::ValReadyToSwitchOn) {
        current_state_ = DriveState::ReadyToSwitchOn;
    } else if ((statusword & CiA402::Statusword::MaskSwitchOnDisabled) == CiA402::Statusword::ValSwitchOnDisabled) {
        current_state_ = DriveState::SwitchOnDisabled;
    } else {
        current_state_ = DriveState::NotReadyToSwitchOn;
    }
}

} // namespace RDT