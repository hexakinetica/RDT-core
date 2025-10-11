// InternalSimulation.h
#ifndef INTERNAL_SIMULATION_H
#define INTERNAL_SIMULATION_H

#pragma once
#include "IHardwareInterface.h"
#include <atomic> // *** NEW: Include for std::atomic ***

namespace RDT {

class InternalSimulation : public IHardwareInterface {
public:
    explicit InternalSimulation(const InterfaceConfig& config);
    ~InternalSimulation() override;

    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    [[nodiscard]] CommStatus sendCommand(const AxisSet& command) override;
    [[nodiscard]] AxisSet getLatestFeedback() const override;
    void setState(const AxisSet& state) override;

private:
    // *** MODIFIED: Using std::atomic for thread-safe state ***
    std::atomic<AxisSet> simulated_joints_state_;
    
    std::atomic<bool> is_connected_{false};
};

} // namespace RDT
#endif // INTERNAL_SIMULATION_H