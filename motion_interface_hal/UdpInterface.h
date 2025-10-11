// UdpInterface.h
#ifndef UDP_INTERFACE_H
#define UDP_INTERFACE_H

#pragma once

#include "IHardwareInterface.h"
#include "UdpPeer.hpp"
#include <thread>
#include <atomic>
#include <string>
#include <sstream>

namespace RDT {

/**
 * @class UdpInterface
 * @brief Implements IHardwareInterface for communication with a robot over UDP.
 *
 * This class sends joint commands as formatted strings to a remote UDP peer.
 * It runs a background thread to continuously listen for feedback packets,
 * parse them, and update the latest known state of the robot.
 */
class UdpInterface : public IHardwareInterface {
public:
    /**
     * @brief Constructs a UdpInterface with the given configuration.
     * @param config The main interface configuration containing UDP settings.
     */
    explicit UdpInterface(const InterfaceConfig& config);
    ~UdpInterface() override;

    // --- IHardwareInterface Implementation ---
    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    [[nodiscard]] CommStatus sendCommand(const AxisSet& command) override;
    [[nodiscard]] AxisSet getLatestFeedback() const override;
    
private:
    /**
     * @brief The main loop for the feedback-receiving thread.
     * @param stoken A stop_token to signal when the thread should exit.
     */
    void feedbackLoop(std::stop_token stoken);

    /**
     * @brief Serializes an AxisSet into a string for UDP transmission.
     * @param axes The AxisSet to serialize.
     * @return A string in the format "A1:val,A2:val,...".
     */
    [[nodiscard]] std::string serialize(const AxisSet& axes) const;

    /**
     * @brief Deserializes a string from a UDP packet into an AxisSet.
     * @param data The received string data.
     * @param[out] axes The AxisSet to populate with parsed data.
     * @return true on successful parsing, false otherwise.
     */
    [[nodiscard]] bool deserialize(const std::string& data, AxisSet& axes) const;

    const UdpConfig config_;                ///< UDP-specific network configuration.
    std::unique_ptr<UdpPeer> udp_peer_;     ///< The underlying UDP socket wrapper.
    std::atomic<bool> is_connected_{false}; ///< Atomic flag for connection status.
    std::jthread feedback_thread_;          ///< Thread for asynchronously receiving feedback.
    
    ///< Thread-safe storage for the most recent feedback from the hardware.
    std::atomic<AxisSet> last_feedback_state_;

    static inline const std::string MODULE_NAME = "UdpInterface";
};

} // namespace RDT
#endif // UDP_INTERFACE_H