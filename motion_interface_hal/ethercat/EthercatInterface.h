// EthercatInterface.h
#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#pragma once
#include "IHardwareInterface.h"
#include "Logger.h"
#include "EthercatAxis.h"

#include <ecrt.h>
#include <vector>
#include <string>
#include <memory>
#include <chrono> 

namespace RDT {

class EthercatInterface : public IHardwareInterface {
public:
    explicit EthercatInterface(const InterfaceConfig& config);
    ~EthercatInterface() override;

    EthercatInterface(const EthercatInterface&) = delete;
    EthercatInterface& operator=(const EthercatInterface&) = delete;
    
    static bool restartService();

    // Реализация IHardwareInterface
    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;
    [[nodiscard]] CommStatus sendCommand(const AxisSet& command) override;
    [[nodiscard]] AxisSet getLatestFeedback() const override;

    // Методы конфигурации (NRT)
    bool configureAxis(int axis_index, const AxisConfiguration& config);
    void resetAxisFault(int axis_index);
    void calibrateAxisZero(int axis_index);
    void enableAxis(int axis_index);
    void disableAxis(int axis_index);
    bool saveCalibration(const std::string& filepath) const;
    bool loadCalibration(const std::string& filepath);
    
    // Методы для работы с SDO (NRT)
    [[nodiscard]] bool readSdo(uint16_t position, uint16_t index, uint8_t subindex, size_t size, uint32_t& value_out);
    bool writeSdo(uint16_t position, uint16_t index, uint8_t subindex, uint32_t value, size_t size);
    
    // Методы для диагностики (NRT)
    [[nodiscard]] int getConnectedSlavesCount() const;
    [[nodiscard]] DriveState getAxisDriveState(int axis_index) const;
    [[nodiscard]] std::string getAxisErrorString(int axis_index) const;
    [[nodiscard]] uint16_t getAxisStatusword(int axis_index) const;
    [[nodiscard]] int32_t getAxisRawEncoderValue(int axis_index) const;
    [[nodiscard]] bool isAxisEnableRequested(int axis_index) const;

private:
    void checkMasterState();
    void checkDomainState();
    void checkSlaveConfigs();
    
    const InterfaceConfig config_;
    bool is_connected_ = false;
    
    ec_master_t* master_ = nullptr;
    ec_domain_t* domain_ = nullptr;
    uint8_t* domain_pd_ = nullptr;
    std::vector<ec_slave_config_t*> slave_configs_;

    std::vector<std::unique_ptr<EthercatAxis>> axes_;
    int connected_slaves_count_ = 0;

    uint64_t cycle_time_ns_ = 0;
    timespec wakeup_time_;
    
    AxisSet last_feedback_state_;

    static inline const std::string MODULE_NAME = "EthercatInterface";
};

} // namespace RDT
#endif // ETHERCAT_INTERFACE_H