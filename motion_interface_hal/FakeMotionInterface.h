// FakeMotionInterface.h
#ifndef FAKEMOTIONINTERFACE_H
#define FAKEMOTIONINTERFACE_H

#include "IMotionInterfaces.h" // Changed include name
#include "DataTypes.h"         // For RDT types
#include "Logger.h"            // For logging
#include <mutex>
#include <chrono>
#include <thread> // For std::this_thread::sleep_for

namespace RDT {

class FakeMotionInterface : public IMotionInterface {
public:
    FakeMotionInterface();
    ~FakeMotionInterface() override;

    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    bool sendCommand(const RobotCommandFrame& cmd) override;
    [[nodiscard]] RobotFeedbackFrame readState() override;
    void reset() override;
    void emergencyStop() override;
    [[nodiscard]] std::string getInterfaceName() const override;
 
private:
    void reset_impl();
    RobotFeedbackFrame current_feedback_state_{}; //   RobotFeedbackFrame
    AxisSet prev_joint_target_; //    joint_target

    
    bool connected_ = false;
    mutable std::mutex state_mutex_; // Mutable to allow locking in const readState
    std::chrono::steady_clock::time_point last_command_time_;

    const std::string MODULE_NAME = "FakeMotion";
};

} // namespace RDT

#endif // FAKEMOTIONINTERFACE_H