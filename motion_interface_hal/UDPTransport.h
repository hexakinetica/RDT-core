// UDPTransport.h
#ifndef UDPTRANSPORT_H
#define UDPTRANSPORT_H

#pragma once
// BEGIN MODIFICATION
#include "ITransport.h" // Include the interface
#include "UdpPeer.hpp"  // For NetworkConfig and UdpPeer
// END MODIFICATION
#include <memory>
#include <vector> // Already there from ITransport implicitly, but good to be explicit if used directly

// BEGIN MODIFICATION
namespace RDT {
// END MODIFICATION

// class UdpPeer; // Forward declaration if UdpPeer.hpp is not included, but it is.

class UDPTransport : public ITransport {
public:
    // BEGIN MODIFICATION
    explicit UDPTransport(NetworkConfig config); // NetworkConfig is now RDT::NetworkConfig
    // END MODIFICATION

    void send(const std::vector<char>& data) override;
    std::vector<char> receive() override;

private:
    std::unique_ptr<UdpPeer> peer_;
};

// BEGIN MODIFICATION
} // namespace RDT
// END MODIFICATION

#endif // UDPTRANSPORT_H