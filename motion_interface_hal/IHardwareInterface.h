// IHardwareInterface.h
#ifndef IHARDWARE_INTERFACE_H
#define IHARDWARE_INTERFACE_H

#pragma once
#include "DataTypes.h"
#include "RobotConfig.h"

namespace RDT {

enum class CommStatus { Success, Error, Timeout, NotConnected };

/**
 * @brief Pure virtual interface for a hardware or simulation layer.
 * Each implementation is responsible for its own background feedback polling.
 */
class IHardwareInterface {
public:
    virtual ~IHardwareInterface() = default;

    /** @brief Establishes connection and starts background threads. */
    virtual bool connect() = 0;
    /** @brief Stops background threads and closes connection. */
    virtual void disconnect() = 0;
    /** @brief Returns true if the interface is connected and running. */
    [[nodiscard]] virtual bool isConnected() const = 0;

    /** @brief Sends a joint position command. */
    [[nodiscard]] virtual CommStatus sendCommand(const AxisSet& command) = 0;
    
    /** @brief Non-blockingly returns the most recent feedback state. */
    [[nodiscard]] virtual AxisSet getLatestFeedback() const = 0;

    /** @brief (For simulation) Forcibly sets the internal state. */
  virtual void setState(const AxisSet& /*state*/) {}
};

} // namespace RDT
#endif // IHARDWARE_INTERFACE_H