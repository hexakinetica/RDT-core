// ITransport.h
#ifndef ITRANSPORT_H
#define ITRANSPORT_H

#pragma once
#include <vector>
#include <string> // For potential error messages or config

namespace RDT {

/**
 * @brief Interface for a generic data transport mechanism (e.g., UDP, TCP, Serial).
 * Responsible for sending and receiving raw byte streams.
 */
class ITransport {
public:
    virtual ~ITransport() = default;

    /**
     * @brief Sends a block of data over the transport.
     * @param data The data to send as a vector of chars.
     * @throws std::runtime_error if sending fails.
     */
    virtual void send(const std::vector<char>& data) = 0;

    /**
     * @brief Receives data from the transport.
     * This method may block until data is received or a timeout occurs.
     * @return std::vector<char> The received data.
     * @throws std::runtime_error if receiving fails or a timeout occurs (if transport implements timeouts).
     *         An empty vector might also indicate a timeout or no data, depending on implementation.
     */
    virtual std::vector<char> receive() = 0;

    // Optional connection management, if the transport layer itself has a connect/disconnect lifecycle.
    // For a stateless UDP send/receive, these might not be strictly necessary at this level,
    // but good for more complex transports.
    // virtual bool connect(const std::string& address, int port) = 0;
    // virtual void disconnect() = 0;
    // virtual bool isConnected() const = 0;
};

} // namespace RDT

#endif // ITRANSPORT_H