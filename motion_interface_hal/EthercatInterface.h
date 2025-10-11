// EthercatInterface.h
#ifndef ETHERCAT_INTERFACE_H
#define ETHERCAT_INTERFACE_H

#pragma once
#include "IHardwareInterface.h"
#include <thread>
#include <atomic>

namespace RDT {

class EthercatInterface : public IHardwareInterface {
public:
    explicit EthercatInterface(const InterfaceConfig& config);
    ~EthercatInterface() override;

    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    [[nodiscard]] CommStatus sendCommand(const AxisSet& command) override;
    [[nodiscard]] AxisSet getLatestFeedback() const override;
    
private:
    // readState is a private implementation detail for the feedback thread
    [[nodiscard]] CommStatus readState(AxisSet& feedback);
    void feedbackLoop(std::stop_token stoken);

    const InterfaceConfig config_;
    std::atomic<bool> is_connected_{false};

    std::jthread feedback_thread_;
    
    // *** MODIFIED: Using std::atomic for thread-safe state ***
    std::atomic<AxisSet> last_feedback_state_;
};

} // namespace RDT
#endif // ETHERCAT_INTERFACE_H