// UdpInterface.cpp
#include "UdpInterface.h"
#include "Logger.h"
#include <vector>

namespace RDT {

using namespace std::chrono_literals;

UdpInterface::UdpInterface(const InterfaceConfig& config)
    : config_(config.udp_control_config) {
    // Initialize the atomic AxisSet
    last_feedback_state_.store(AxisSet{});
}

UdpInterface::~UdpInterface() {
    disconnect();
}

bool UdpInterface::connect() {
    if (is_connected_.load()) {
        return true;
    }

    LOG_INFO(MODULE_NAME, "UDP interface connecting...");

    NetworkConfig net_conf;
    net_conf.remote_ip = config_.remote_ip;
    net_conf.remote_port = config_.remote_port;
    net_conf.local_port = config_.local_port;
    net_conf.receive_timeout_ms = config_.timeout_ms;
    
    udp_peer_ = std::make_unique<UdpPeer>(net_conf);
    if (udp_peer_->connect() != 0) {
        LOG_CRITICAL_F(MODULE_NAME, "Failed to create or bind UDP socket on local port %u.", config_.local_port);
        udp_peer_.reset();
        return false;
    }

    is_connected_.store(true);
    
    feedback_thread_ = std::jthread([this](std::stop_token stoken) {
        this->feedbackLoop(stoken);
    });

    LOG_INFO_F(MODULE_NAME, "UDP interface connected. Sending to %s:%u, listening on port %u.",
               config_.remote_ip.c_str(), config_.remote_port, config_.local_port);
    return true;
}

void UdpInterface::disconnect() {
    if (!is_connected_.load()) {
        return;
    }
    is_connected_.store(false);
    
    if (feedback_thread_.joinable()) {
        feedback_thread_.request_stop();
        feedback_thread_.join();
    }
    
    if (udp_peer_) {
        udp_peer_->disconnect();
        udp_peer_.reset();
    }
    LOG_INFO(MODULE_NAME, "UDP interface disconnected.");
}

bool UdpInterface::isConnected() const {
    return is_connected_.load();
}

CommStatus UdpInterface::sendCommand(const AxisSet& command) {
    if (!isConnected() || !udp_peer_) {
        return CommStatus::NotConnected;
    }

    std::string payload_str = serialize(command);
    std::vector<char> payload(payload_str.begin(), payload_str.end());

    if (udp_peer_->send(payload) < 0) {
        LOG_WARN(MODULE_NAME, "Failed to send UDP command packet.");
        return CommStatus::Error;
    }
    
    LOG_DEBUG_F(MODULE_NAME, "Sent command: %s", payload_str.c_str());
    return CommStatus::Success;
}

AxisSet UdpInterface::getLatestFeedback() const {
    return last_feedback_state_.load();
}

void UdpInterface::feedbackLoop(std::stop_token stoken) {
    LOG_INFO(MODULE_NAME, "Feedback thread started.");
    std::vector<char> buffer;
    AxisSet received_state;

    while (!stoken.stop_requested()) {
        if (!isConnected() || !udp_peer_) {
            std::this_thread::sleep_for(100ms);
            continue;
        }

        int bytes_received = udp_peer_->receive(buffer);
        if (bytes_received > 0) {
            std::string data_str(buffer.begin(), buffer.end());
            if (deserialize(data_str, received_state)) {
                last_feedback_state_.store(received_state);
                LOG_DEBUG_F(MODULE_NAME, "Received feedback: %s", data_str.c_str());
            } else {
                LOG_WARN_F(MODULE_NAME, "Failed to parse feedback packet: %s", data_str.c_str());
            }
        } else if (bytes_received == -2) {
            // This is a timeout, which is expected. No log needed to avoid spam.
        } else {
            LOG_ERROR_F(MODULE_NAME, "UDP receive error (code %d).", bytes_received);
            // In a real scenario, you might want a mechanism to signal a connection loss.
        }
    }
    LOG_INFO(MODULE_NAME, "Feedback thread stopped.");
}

std::string UdpInterface::serialize(const AxisSet& axes) const {
    std::ostringstream ss;
    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        ss << "A" << (i + 1) << ":" << std::fixed << std::setprecision(4)
           << axes.at(i).angle.toDegrees().value();
        if (i < ROBOT_AXES_COUNT - 1) {
            ss << ",";
        }
    }
    return ss.str();
}

bool UdpInterface::deserialize(const std::string& data, AxisSet& axes) const {
    std::stringstream ss(data);
    std::string segment;
    int parsed_count = 0;

    while(std::getline(ss, segment, ',')) {
        std::size_t colon_pos = segment.find(':');
        if (colon_pos == std::string::npos || segment.length() < 3) {
            continue; // Invalid segment
        }
        
        std::string key = segment.substr(0, colon_pos);
        std::string value_str = segment.substr(colon_pos + 1);

        if (key[0] == 'A') {
            try {
                int axis_idx = std::stoi(key.substr(1)) - 1;
                if (axis_idx >= 0 && axis_idx < static_cast<int>(ROBOT_AXES_COUNT)) {
                    double angle_deg = std::stod(value_str);
                    axes.at(axis_idx).angle = Degrees(angle_deg).toRadians();
                    parsed_count++;
                }
            } catch (const std::exception&) {
                // Ignore parsing errors for individual segments
                continue;
            }
        }
    }
    // Succeed if at least one axis was parsed correctly.
    return parsed_count > 0;
}

} // namespace RDT