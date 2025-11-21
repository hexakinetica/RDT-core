// EthercatInterface.cpp
#include "EthercatInterface.h"
#include "Logger.h"
#include <chrono>
#include <thread>
#include <fstream>
#include <iostream>

#include <sys/mman.h>
#include <time.h>

namespace RDT {

using namespace std::chrono_literals;

EthercatInterface::EthercatInterface(const InterfaceConfig& config)
    : config_(config) {
    last_feedback_state_ = {};
}

EthercatInterface::~EthercatInterface() {
    disconnect();
}

bool EthercatInterface::connect() {
    if (is_connected_) return true;
    master_ = ecrt_request_master(0);
    if (!master_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to request EtherCAT master 0.");
        return false;
    }
    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to create master domain.");
        ecrt_release_master(master_);
        master_ = nullptr;
        return false;
    }
    ec_master_info_t master_info{};
    if (ecrt_master(master_, &master_info)) {
        LOG_CRITICAL(MODULE_NAME, "Failed to get master info.");
        disconnect();
        return false;
    }
    LOG_INFO_F(MODULE_NAME, "Master scan found %u slaves.", master_info.slave_count);
    
    axes_.resize(ROBOT_AXES_COUNT);
    slave_configs_.resize(ROBOT_AXES_COUNT);
    for (uint16_t i = 0; i < master_info.slave_count && i < ROBOT_AXES_COUNT; ++i) {
        ec_slave_info_t slave_info{};
        if (ecrt_master_get_slave(master_, i, &slave_info) != 0) continue;

        if (slave_info.vendor_id == CiA402::DVS_VENDOR_ID && slave_info.product_code == CiA402::DVS_PRODUCT_CODE) {
            LOG_INFO_F(MODULE_NAME, "Found compatible DVS Drive at bus position %u.", i);
            ec_slave_config_t* sc = ecrt_master_slave_config(master_, 0, i, CiA402::DVS_VENDOR_ID, CiA402::DVS_PRODUCT_CODE);
            if (!sc) {
                LOG_ERROR_F(MODULE_NAME, "Failed to get slave config for pos %u.", i);
                continue;
            }
            if (ecrt_slave_config_pdos(sc, EC_END, CiA402::Pdo::syncs)) {
                LOG_ERROR_F(MODULE_NAME, "Failed to configure PDOs for slave %u.", i);
                continue;
            }
            
            axes_[i] = std::make_unique<EthercatAxis>(0, i, CiA402::DVS_VENDOR_ID, CiA402::DVS_PRODUCT_CODE);
            axes_[i]->registerPdosInDomain(domain_);
            slave_configs_[i] = sc;
            connected_slaves_count_++;
        }
    }
    if (connected_slaves_count_ == 0) {
        LOG_CRITICAL(MODULE_NAME, "No compatible EtherCAT slaves found.");
        disconnect();
        return false;
    }
    
    LOG_INFO(MODULE_NAME, "Activating EtherCAT master...");
    cycle_time_ns_ = config_.motion_manager_cycle_ms * 1'000'000;
    if (ecrt_master_activate(master_)) {
        LOG_CRITICAL(MODULE_NAME, "Failed to activate master.");
        disconnect();
        return false;
    }
    
    domain_pd_ = ecrt_domain_data(domain_);
    if (!domain_pd_) {
        LOG_CRITICAL(MODULE_NAME, "Failed to get domain data pointer.");
        disconnect();
        return false;
    }
    
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time_);
    is_connected_ = true;
    LOG_INFO_F(MODULE_NAME, "EtherCAT interface connected. %d slaves active.", connected_slaves_count_);
    return true;
}

void EthercatInterface::disconnect() {
    if (!is_connected_) return;
    is_connected_ = false;
    std::this_thread::sleep_for(std::chrono::milliseconds(100)); // Дать циклу завершиться
    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
    axes_.clear();
    slave_configs_.clear();
    connected_slaves_count_ = 0;
    LOG_INFO(MODULE_NAME, "EtherCAT interface disconnected.");
}

bool EthercatInterface::isConnected() const {
    return is_connected_;
}

CommStatus EthercatInterface::sendCommand(const AxisSet& command) {
    if (!isConnected()) { return CommStatus::NotConnected; }

    // Ожидание следующего цикла
    wakeup_time_.tv_nsec += cycle_time_ns_;
    while (wakeup_time_.tv_nsec >= 1000000000) {
        wakeup_time_.tv_nsec -= 1000000000;
        wakeup_time_.tv_sec++;
    }
    clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time_, NULL);

    ecrt_master_receive(master_);
    ecrt_domain_process(domain_);
    
    // УПРОЩЕНО: вызовы пустых check-функций убраны из RT-цикла
    
    AxisSet current_feedback;
    for (int i = 0; i < connected_slaves_count_; ++i) {
        if (axes_[i]) {
            axes_[i]->processFeedback(domain_pd_);
            axes_[i]->writeCommand(domain_pd_, command.at(i).angle);
            current_feedback.at(i) = axes_[i]->getActualState();
        }
    }
    
    last_feedback_state_ = current_feedback;

    ecrt_domain_queue(domain_);
    ecrt_master_send(master_);
    
    return CommStatus::Success;
}

AxisSet EthercatInterface::getLatestFeedback() const {
    return last_feedback_state_;
}

bool EthercatInterface::configureAxis(int axis_index, const AxisConfiguration& config) {
    if (!master_ || axis_index < 0 || axis_index >= connected_slaves_count_ || !axes_[axis_index]) {
        return false;
    }
    axes_[axis_index]->configure(slave_configs_[axis_index], config);
    return true;
}

void EthercatInterface::resetAxisFault(int axis_index) {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) {
        axes_[axis_index]->requestFaultReset();
    }
}

void EthercatInterface::calibrateAxisZero(int axis_index) {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) {
        axes_[axis_index]->requestZeroCalibration();
    }
}

void EthercatInterface::enableAxis(int axis_index) {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) {
        axes_[axis_index]->requestEnable();
    }
}

void EthercatInterface::disableAxis(int axis_index) {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) {
        axes_[axis_index]->requestDisable();
    }
}

bool EthercatInterface::saveCalibration(const std::string& filepath) const {
    std::ofstream file(filepath);
    if (!file.is_open()) {
        LOG_ERROR_F(MODULE_NAME, "Failed to open calibration file for writing: %s", filepath.c_str());
        return false;
    }
    for (int i = 0; i < connected_slaves_count_; ++i) {
        if (axes_[i]) {
            file << i << ":" << axes_[i]->getZeroOffset() << "\n";
        }
    }
    LOG_INFO_F(MODULE_NAME, "Calibration data saved to %s", filepath.c_str());
    return true;
}

bool EthercatInterface::loadCalibration(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        LOG_WARN_F(MODULE_NAME, "Calibration file not found: %s. Using default zero offsets.", filepath.c_str());
        return false;
    }
    std::string line;
    while (std::getline(file, line)) {
        size_t colon_pos = line.find(':');
        if (colon_pos != std::string::npos) {
            try {
                int axis_idx = std::stoi(line.substr(0, colon_pos));
                int32_t offset = std::stoi(line.substr(colon_pos + 1));
                if (axis_idx >= 0 && axis_idx < connected_slaves_count_ && axes_[axis_idx]) {
                    axes_[axis_idx]->setZeroOffset(offset);
                    LOG_INFO_F(MODULE_NAME, "Loaded calibration for axis %d: offset %d", axis_idx, offset);
                }
            } catch (const std::exception& e) {
                LOG_ERROR_F(MODULE_NAME, "Error parsing calibration line: '%s'", line.c_str());
            }
        }
    }
    return true;
}

// ... SDO и get-методы без изменений ...
bool EthercatInterface::readSdo(uint16_t position, uint16_t index, uint8_t subindex, size_t size, uint32_t& value_out) {
    if (!is_connected_ || position >= connected_slaves_count_) return false;
    
    uint32_t abort_code;
    size_t result_size = size;

    int ret = ecrt_master_sdo_upload(master_, position, index, subindex,
                                     reinterpret_cast<uint8_t*>(&value_out), size, &result_size, &abort_code);

    if (ret < 0) {
        LOG_ERROR_F(MODULE_NAME, "SDO upload for slave %u failed with lib error %d.", position, ret);
        return false;
    }
    if (abort_code) {
        LOG_ERROR_F(MODULE_NAME, "SDO upload for slave %u aborted with code 0x%08X.", position, abort_code);
        return false;
    }

    return true;
}

bool EthercatInterface::writeSdo(uint16_t position, uint16_t index, uint8_t subindex, uint32_t value, size_t size) {
    if (!is_connected_ || position >= connected_slaves_count_) return false;

    uint32_t abort_code;
    int ret = ecrt_master_sdo_download(master_, position, index, subindex,
                                       reinterpret_cast<uint8_t*>(&value), size, &abort_code);

    if (ret < 0) {
        LOG_ERROR_F(MODULE_NAME, "SDO download for slave %u failed with lib error %d.", position, ret);
        return false;
    }
    if (abort_code) {
        LOG_ERROR_F(MODULE_NAME, "SDO download for slave %u aborted with code 0x%08X.", position, abort_code);
        return false;
    }

    return true;
}

int EthercatInterface::getConnectedSlavesCount() const { return connected_slaves_count_; }
DriveState EthercatInterface::getAxisDriveState(int axis_index) const {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) return axes_[axis_index]->getDriveState();
    return DriveState::NotReadyToSwitchOn;
}
std::string EthercatInterface::getAxisErrorString(int axis_index) const {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) return axes_[axis_index]->getErrorString();
    return "Axis not available";
}
uint16_t EthercatInterface::getAxisStatusword(int axis_index) const {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) return axes_[axis_index]->getStatusword();
    return 0;
}
int32_t EthercatInterface::getAxisRawEncoderValue(int axis_index) const {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) return axes_[axis_index]->getRawEncoderValue();
    return 0;
}
bool EthercatInterface::isAxisEnableRequested(int axis_index) const {
    if (axis_index >= 0 && axis_index < connected_slaves_count_ && axes_[axis_index]) return axes_[axis_index]->isEnableRequested();
    return false;
}

void EthercatInterface::checkMasterState() {
    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);
    // Для отладки можно добавить:
    // if (ms.slaves_responding != connected_slaves_count_) ...
}
void EthercatInterface::checkDomainState() {
    ec_domain_state_t ds;
    ecrt_domain_state(domain_, &ds);
    // if (ds.wc_state != EC_WC_COMPLETE) ...
}
void EthercatInterface::checkSlaveConfigs() {
    // Эта функция полезна для диагностики на этапе ПОСЛЕ активации мастера.
    for (int i = 0; i < connected_slaves_count_; ++i) {
        ec_slave_config_state_t scs;
        ecrt_slave_config_state(slave_configs_[i], &scs);
        // if (!scs.operational) ...
    }
}

} // namespace RDT