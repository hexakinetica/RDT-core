// UDPMotionInterface.h
#ifndef UDPMOTIONINTERFACE_H
#define UDPMOTIONINTERFACE_H

#pragma once
// BEGIN MODIFICATION
#include "IMotionInterfaces.h" // Contains RDT::IMotionInterface
#include "ITransport.h"      // Contains RDT::ITransport
#include "DataTypes.h"       // Contains RDT::RobotCommandFrame etc.
#include "Logger.h"
#include "tinyxml2.h"        // Include full header for tinyxml2 types
// END MODIFICATION
#include <memory>
#include <vector>
#include <mutex>

// namespace tinyxml2 { class XMLDocument; class XMLElement; } // Not needed if tinyxml2.h is included

// BEGIN MODIFICATION
namespace RDT {
// END MODIFICATION

class UDPMotionInterface : public IMotionInterface {
public:
    explicit UDPMotionInterface(std::unique_ptr<ITransport> transport);
    ~UDPMotionInterface() override;

    bool connect() override;
    void disconnect() override;
    [[nodiscard]] bool isConnected() const override;

    bool sendCommand(const RobotCommandFrame& cmd) override;
    [[nodiscard]] RobotFeedbackFrame readState() override;
    void reset() override;
    void emergencyStop() override;
    [[nodiscard]] std::string getInterfaceName() const override;

private:
    void reset_impl(); // Private method without lock

    std::unique_ptr<ITransport> transport_;
    RobotFeedbackFrame last_received_feedback_{};
    bool connected_ = false;
    mutable std::mutex interface_mutex_;

    void serializeCommandToXML(const RobotCommandFrame& cmd, std::vector<char>& out_buffer);
    bool deserializeFeedbackFromXML(const std::vector<char>& in_buffer, RobotFeedbackFrame& out_frame);

    // Ensure tinyxml2 namespace is used for its types
    void addAxisSetToXML(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* parent, const AxisSet& axes, const std::string& setName);
    bool parseAxisSetFromXML(tinyxml2::XMLElement* parent, AxisSet& out_axes, const std::string& setName);

    const std::string MODULE_NAME = "UDPMotion";
};

// BEGIN MODIFICATION
} // namespace RDT
// END MODIFICATION
#endif // UDPMOTIONINTERFACE_H