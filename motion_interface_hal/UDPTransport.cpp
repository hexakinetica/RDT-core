// UDPTransport.cpp
// BEGIN MODIFICATION
#include "UDPTransport.h"
#include "Logger.h" // Assuming Logger.h is available and in RDT namespace
// END MODIFICATION
#include <stdexcept>

// BEGIN MODIFICATION
namespace RDT {
// END MODIFICATION

// Constructor takes RDT::NetworkConfig due to UdpPeer.hpp change
UDPTransport::UDPTransport(NetworkConfig config) { // NetworkConfig is RDT::NetworkConfig
    peer_ = std::make_unique<UdpPeer>(std::move(config));
    if (peer_->connect() != 0) {
        // BEGIN MODIFICATION
        LOG_CRITICAL_F("UDPTransport", "Failed to connect UDP peer. Error code from peer: %d (Check UdpPeer codes)", peer_->connect()); // Log before throwing
        // END MODIFICATION
        throw std::runtime_error("Failed to connect UDP peer");
    }
    // BEGIN MODIFICATION
    LOG_INFO_F("UDPTransport", "UDPTransport connected. LocalPort: %hu, Remote: %s:%hu", config.local_port, config.remote_ip.c_str(), config.remote_port);
    // END MODIFICATION
}

void UDPTransport::send(const std::vector<char>& data) {
    int sent_bytes = peer_->send(data);
    // BEGIN MODIFICATION
    if (sent_bytes < 0) {
        LOG_ERROR_F("UDPTransport", "Failed to send UDP packet, peer_send_code: %d", sent_bytes);
    // END MODIFICATION
        throw std::runtime_error("Failed to send UDP packet");
    }
    // BEGIN MODIFICATION
    LOG_DEBUG_F("UDPTransport", "Sent %d bytes.", sent_bytes);
    // END MODIFICATION
}

std::vector<char> UDPTransport::receive() {
    std::vector<char> buffer;
    // BEGIN MODIFICATION
    int received_bytes = peer_->receive(buffer); // Handle nodiscard
    if (received_bytes < 0) {
        // -2 from UdpPeer::receive usually means timeout or actual recvfrom error
        if (received_bytes == -2) { // Assuming -2 is timeout/error from UdpPeer's recvfrom
             LOG_WARN_F("UDPTransport", "UDP receive timeout or error, peer_recv_code: %d", received_bytes);
        } else {
             LOG_ERROR_F("UDPTransport", "Failed to receive UDP packet, peer_recv_code: %d", received_bytes);
        }
        // Depending on policy, either throw or return empty vector
        // For now, let's throw on error, but return empty on typical timeout if that's distinguishable
        // UdpPeer's current `receive` returns -2 for any recvfrom error including timeout.
        // throw std::runtime_error("Failed to receive UDP packet or timeout");
        return {}; // Return empty on failure/timeout
    }
    if (received_bytes == 0) {
        LOG_WARN("UDPTransport", "Received empty UDP packet (0 bytes), possibly connection closed by peer.");
    } else {
        LOG_DEBUG_F("UDPTransport", "Received %d bytes.", received_bytes);
    }
    // Buffer is already resized by peer_->receive()
    // END MODIFICATION
    return buffer;
}

// BEGIN MODIFICATION
} // namespace RDT
// END MODIFICATION