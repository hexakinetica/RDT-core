// IMotionInterfaces.h
#ifndef IMOTION_INTERFACES_H
#define IMOTION_INTERFACES_H

#pragma once
#include "DataTypes.h" // Uses RDT::RobotCommandFrame, RDT::RobotFeedbackFrame
#include <string>

namespace RDT {

/**
 * @brief Interface for a motion control device (real robot or simulator).
 * Defines the contract for controlling and querying a motion system.
 */
class IMotionInterface {
public:
    virtual ~IMotionInterface() = default;

    /**
     * @brief Attempts to establish a connection with the motion device.
     * @return true if connection was successful or is already established, false otherwise.
     */
    virtual bool connect() = 0;

    /**
     * @brief Terminates the connection with the motion device.
     */
    virtual void disconnect() = 0;

    /**
     * @brief Checks if the motion device is currently connected and operational.
     * @return true if connected and responsive, false otherwise.
     */
    [[nodiscard]] virtual bool isConnected() const = 0;

    /**
     * @brief Sends a command frame to the motion device.
     * @param cmd The RobotCommandFrame to send.
     * @return true if the command was successfully transmitted or queued by the interface.
     *         Does not guarantee the robot has started or completed the command.
     *         Returns false if not connected or an immediate send error occurs.
     */
    virtual bool sendCommand(const RobotCommandFrame& cmd) = 0;

    /**
     * @brief Reads the current state and feedback from the motion device.
     * @return RobotFeedbackFrame containing the latest available feedback.
     * @throws std::runtime_error if reading state fails (e.g., communication error, timeout, parsing error).
     */
    [[nodiscard]] virtual RobotFeedbackFrame readState() = 0;

    /**
     * @brief Resets the motion interface and/or the connected device to a known initial state.
     * The exact behavior is implementation-dependent.
     */
    virtual void reset() = 0;

    /**
     * @brief Activates an emergency stop condition on the motion device.
     * This should halt all motion as quickly and safely as possible.
     */
    virtual void emergencyStop() = 0;

    /**
     * @brief Gets a human-readable name for this specific motion interface implementation.
     * Useful for logging and diagnostics.
     * @return std::string The name of the interface (e.g., "FakeMotionSimulator", "KUKA KR6 UDP Interface").
     */
    [[nodiscard]] virtual std::string getInterfaceName() const = 0;
};

} // namespace RDT

#endif // IMOTION_INTERFACES_H