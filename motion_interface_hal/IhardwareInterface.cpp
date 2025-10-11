// EthercatInterface.cpp
#include "EthercatInterface.h"
#include "Logger.h"

namespace RDT {

using namespace std::chrono_literals;

EthercatInterface::EthercatInterface(const InterfaceConfig& config)
    : config_(config) {}

EthercatInterface::~EthercatInterface() {
    disconnect();
}

bool EthercatInterface::connect() {
    if (is_connected_.load()) return true;

    LOG_INFO("EtherCAT", "EtherCAT interface STUB connecting...");
    // TODO: Real EtherCAT stack initialization would happen here
    is_connected_.store(true);
    
    // *** MODIFIED: Use a lambda to correctly capture 'this' and pass the stop_token ***
    feedback_thread_ = std::jthread([this](std::stop_token stoken) {
        this->feedbackLoop(stoken);
    });

    LOG_INFO_F("EtherCAT", "EtherCAT interface STUB connected on iface %s.", config_.ethercat_config.iface_name.c_str());
    return true;
}

void EthercatInterface::disconnect() {
    is_connected_.store(false);
    if (feedback_thread_.joinable()) {
        feedback_thread_.request_stop();
        feedback_thread_.join();
    }
    LOG_INFO("EtherCAT", "EtherCAT interface STUB disconnected.");
}

bool EthercatInterface::isConnected() const {
    return is_connected_.load();
}

CommStatus EthercatInterface::sendCommand(const AxisSet& command) {
    if (!isConnected()) return CommStatus::NotConnected;
    // TODO: Real EtherCAT command sending (e.g., writing to PDO)
    LOG_DEBUG_F("EtherCAT", "STUB: Sending command for A1: %.4f deg", command.at(0).angle.toDegrees().value());
    return CommStatus::Success;
}

CommStatus EthercatInterface::readState(AxisSet& feedback) {
    if (!isConnected()) return CommStatus::NotConnected;
    // TODO: Real EtherCAT state reading (e.g., reading from PDO)
    // For this stub, we return a fixed, predictable value.
    feedback.at(0).angle = (10.0_deg).toRadians();
    feedback.at(1).angle = (20.0_deg).toRadians();
    feedback.at(2).angle = (30.0_deg).toRadians();
    return CommStatus::Success;
}

AxisSet EthercatInterface::getLatestFeedback() const {
    std::lock_guard<std::mutex> lock(feedback_mutex_);
    return last_feedback_state_;
}

void EthercatInterface::feedbackLoop(std::stop_token stoken) {
    LOG_INFO("EtherCAT", "Feedback thread started.");
    AxisSet current_feedback;
    while (!stoken.stop_requested()) {
        if (readState(current_feedback) == CommStatus::Success) {
            std::lock_guard<std::mutex> lock(feedback_mutex_);
            last_feedback_state_ = current_feedback;
        }
        // This sleep determines the feedback polling rate
        std::this_thread::sleep_for(5ms); 
    }
    LOG_INFO("EtherCAT", "Feedback thread stopped.");
}

} // namespace RDT