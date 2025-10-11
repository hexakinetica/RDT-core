// MasterHardwareInterface.h
#ifndef MASTER_HARDWARE_INTERFACE_H
#define MASTER_HARDWARE_INTERFACE_H

#pragma once

#include "IHardwareInterface.h"
#include "UdpPeer.hpp"
#include <memory>
#include <atomic>
#include <thread>
#include <string>

namespace RDT {

class MasterHardwareInterface : public IHardwareInterface {
public:
    explicit MasterHardwareInterface(const InterfaceConfig& config);
    ~MasterHardwareInterface() override;

    // --- Реализация основного интерфейса IHardwareInterface ---
    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;
    [[nodiscard]] CommStatus sendCommand(const AxisSet& command) override;
    [[nodiscard]] AxisSet getLatestFeedback() const override;
    void setState(const AxisSet& state) override;
    
    // Этот метод не является частью публичного интерфейса для потребителей
    // [[nodiscard]] CommStatus readState(AxisSet& feedback) override; 

    // --- Методы для управления режимами ---
    enum class ActiveMode { Simulation, Realtime };

    struct SyncInfo {
        bool is_in_sync;
        double max_deviation_deg;
        AxisSet simulated_joints;
        AxisSet real_joints;
    };

    [[nodiscard]] SyncInfo checkSync(Degrees tolerance);
    void forceSyncSimulationToReal();

    enum class SwitchResult { Success, NotInSync, RealtimeNotAvailable };
    [[nodiscard]] SwitchResult switchTo(ActiveMode mode);
    [[nodiscard]] ActiveMode getCurrentMode() const;
    bool isRealInterfaceConnected() const;

private:
    void debugStreamThreadLoop(std::stop_token stoken);

    std::unique_ptr<IHardwareInterface> sim_interface_;
    std::unique_ptr<IHardwareInterface> real_interface_;
    
    std::atomic<ActiveMode> active_mode_{ActiveMode::Simulation};
    const InterfaceConfig config_;

    std::unique_ptr<UdpPeer> debug_udp_peer_;
    std::jthread debug_stream_thread_;
    
    static inline const std::string MODULE_NAME = "MasterHwIface";
};

} // namespace RDT
#endif // MASTER_HARDWARE_INTERFACE_H