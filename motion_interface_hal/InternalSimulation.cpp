// InternalSimulation.cpp
#include "InternalSimulation.h"
#include "Logger.h"

namespace RDT {

InternalSimulation::InternalSimulation(const InterfaceConfig& config) {
    // std::atomic must be initialized, store() is the way
    simulated_joints_state_.store(config.simulation_initial_joints);
}

InternalSimulation::~InternalSimulation() {
    disconnect();
}

bool InternalSimulation::connect() {
    is_connected_.store(true);
    LOG_INFO("InternalSim", "Simulation interface connected (virtual).");
    return true;
}

void InternalSimulation::disconnect() {
    is_connected_.store(false);
    LOG_INFO("InternalSim", "Simulation interface disconnected (virtual).");
}

bool InternalSimulation::isConnected() const {
    return is_connected_.load();
}

CommStatus InternalSimulation::sendCommand(const AxisSet& command) {
    if (!is_connected_.load()) return CommStatus::NotConnected;
    
    // *** MODIFIED: Use store() for atomic write ***
    simulated_joints_state_.store(command);
    return CommStatus::Success;
}

AxisSet InternalSimulation::getLatestFeedback() const {
    // *** MODIFIED: Use load() for atomic read ***
    return simulated_joints_state_.load();
}

void InternalSimulation::setState(const AxisSet& state) {
    // *** MODIFIED: Use store() for atomic write ***
    simulated_joints_state_.store(state);
    LOG_INFO("InternalSim", "Simulation state has been synchronized.");
}

} // namespace RDT