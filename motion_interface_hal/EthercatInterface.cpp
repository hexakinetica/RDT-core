// EthercatInterface.cpp
#include "EthercatInterface.h"
#include "Logger.h"

namespace RDT {

using namespace std::chrono_literals;

EthercatInterface::EthercatInterface(const InterfaceConfig& config)
    : config_(config) {
    // Initialize atomic variable
    last_feedback_state_.store(AxisSet{});
}

EthercatInterface::~EthercatInterface() {
    disconnect();
}

bool EthercatInterface::connect() {
    if (is_connected_.load()) return true;

    LOG_INFO("EtherCAT", "EtherCAT interface STUB connecting...");
    // TODO: Real EtherCAT stack initialization
    is_connected_.store(true);
    
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
    // TODO: Real EtherCAT command sending
    LOG_DEBUG_F("EtherCAT", "STUB: Sending command for A1: %.4f deg", command.at(0).angle.toDegrees().value());
    return CommStatus::Success;
}

CommStatus EthercatInterface::readState(AxisSet& feedback) {
    if (!isConnected()) return CommStatus::NotConnected;
    // TODO: Real EtherCAT state reading
    feedback.at(0).angle = (10.0_deg).toRadians();
    feedback.at(1).angle = (20.0_deg).toRadians();
    feedback.at(2).angle = (30.0_deg).toRadians();
    feedback.at(3).angle = 0.0_rad;
    feedback.at(4).angle = 0.0_rad;
    feedback.at(5).angle = 0.0_rad;
    return CommStatus::Success;
}

AxisSet EthercatInterface::getLatestFeedback() const {
    // *** MODIFIED: Use load() for atomic read ***
    return last_feedback_state_.load();
}

void EthercatInterface::feedbackLoop(std::stop_token stoken) {
    LOG_INFO("EtherCAT", "Feedback thread started.");
    AxisSet current_feedback;
    while (!stoken.stop_requested()) {
        if (readState(current_feedback) == CommStatus::Success) {
            // *** MODIFIED: Use store() for atomic write ***
            last_feedback_state_.store(current_feedback);
        }
        std::this_thread::sleep_for(5ms); 
    }
    LOG_INFO("EtherCAT", "Feedback thread stopped.");
}

} // namespace RDT