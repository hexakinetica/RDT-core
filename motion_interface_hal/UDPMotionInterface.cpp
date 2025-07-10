// UDPMotionInterface.cpp
#include "UDPMotionInterface.h"
#include "tinyxml2.h" // XML parsing
#include <sstream>
#include <iomanip>   // For std::fixed, std::setprecision
#include <stdexcept> // For std::runtime_error
#include <chrono>    // For timestamps

// Ensure our RDT::literals are available for convenience if needed
// And RDT::to_string for AxisId
namespace RDT {
using namespace RDT::literals;
}


namespace RDT {

UDPMotionInterface::UDPMotionInterface(std::unique_ptr<ITransport> transport)
    : transport_(std::move(transport)) {
    if (!transport_) {
        throw std::invalid_argument("ITransport cannot be null for UDPMotionInterface.");
    }
    LOG_INFO(MODULE_NAME, "UDPMotionInterface created.");
}

UDPMotionInterface::~UDPMotionInterface() {
    LOG_INFO(MODULE_NAME, "UDPMotionInterface destroyed.");
    if (connected_) {
        disconnect();
    }
}

//  ,  . ,    .
void UDPMotionInterface::reset_impl() {
    //  : std::lock_guard<std::mutex> lock(interface_mutex_);
    LOG_INFO(MODULE_NAME, "Reset command initiated (impl).");
    last_received_feedback_ = RobotFeedbackFrame{};
    // connected_       (connect/disconnect)
    //   reset_impl.
    if (connected_) { 
        last_received_feedback_.rt_state = RTState::Initializing;
    } else {
        //      , ,     
        last_received_feedback_.rt_state = RTState::Error; //  Idle
    }
    LOG_INFO(MODULE_NAME, "Internal state reset (impl). Robot might be re-initializing.");
}


bool UDPMotionInterface::connect() {
    std::lock_guard<std::mutex> lock(interface_mutex_); //  
    if (connected_) {
        LOG_WARN(MODULE_NAME, "Already connected.");
        return true;
    }
    LOG_INFO(MODULE_NAME, "Attempting to establish UDP communication channel...");
    try {
        //       ,  ITransport  ,
        //      "ping"   "pong".
        //  ,  transport_ ,   " ".
        //   isConnected()     send/receive.
        
        connected_ = true; //  connected_   reset_impl
        LOG_INFO(MODULE_NAME, "UDP communication channel marked active.");
        reset_impl();      //  reset_impl,    
        
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE_NAME, "Connection attempt failed during setup: " + std::string(e.what()));
        connected_ = false; // ,       connected_  
        return false;
    }
}

void UDPMotionInterface::disconnect() {
    std::lock_guard<std::mutex> lock(interface_mutex_);
    if (!connected_) {
        LOG_WARN(MODULE_NAME, "Not connected.");
        return;
    }
    LOG_INFO(MODULE_NAME, "Closing UDP communication channel...");
    //  ITransport   disconnect,  :
    // if (transport_ && transport_->isConnected()) transport_->disconnect(); 
    connected_ = false;
    //    reset_impl() ,   last_received_feedback_
    //     "".
    last_received_feedback_.rt_state = RTState::Idle; //   ,  " "
    LOG_INFO(MODULE_NAME, "UDP communication channel closed.");
}


//   reset     
void UDPMotionInterface::reset() {
    std::lock_guard<std::mutex> lock(interface_mutex_);
    reset_impl(); //  
}


bool UDPMotionInterface::isConnected() const {
    // std::lock_guard<std::mutex> lock(interface_mutex_); // If transport itself is not thread-safe
    return connected_; // And potentially: && transport_ && transport_->isConnected();
}

bool UDPMotionInterface::sendCommand(const RobotCommandFrame& cmd) {
    std::lock_guard<std::mutex> lock(interface_mutex_);
    if (!connected_) {
        LOG_ERROR(MODULE_NAME, "Cannot send command: Not connected.");
        return false;
    }

    LOG_DEBUG(MODULE_NAME, "Serializing command: Cart(" + cmd.cartesian_target.toDescriptiveString() +
                           "), Joint(" + cmd.joint_target.toJointPoseString() + ")");
    std::vector<char> buffer;
    try {
        serializeCommandToXML(cmd, buffer);
        transport_->send(buffer);
        LOG_DEBUG(MODULE_NAME, "Command sent (" + std::to_string(buffer.size()) + " bytes).");
        return true;
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE_NAME, "Failed to send command: " + std::string(e.what()));
        // Optionally set connected_ to false if send implies connection loss
        return false;
    }
}

RobotFeedbackFrame UDPMotionInterface::readState() {
    std::lock_guard<std::mutex> lock(interface_mutex_);
    if (!connected_) {
        LOG_WARN(MODULE_NAME, "Cannot read state: Not connected. Returning last known or default.");
        // Return a default or last known state with an error/disconnected indication
        RobotFeedbackFrame disconnected_state;
        disconnected_state.rt_state = RTState::Error; // Or a specific "Offline" state
        return disconnected_state;
    }

    try {
        std::vector<char> data = transport_->receive(); // This might block or timeout
        if (data.empty()) {
            LOG_WARN(MODULE_NAME, "Received empty data packet (timeout or no data).");
            // Potentially update connection status or return last_received_feedback_
            // with a timestamp to indicate staleness.
            // For now, assume timeout = error or stale data.
             throw std::runtime_error("Receive timeout or empty packet");
        }

        LOG_DEBUG(MODULE_NAME, "Received data (" + std::to_string(data.size()) + " bytes). Deserializing...");
        RobotFeedbackFrame new_feedback;
        if (!deserializeFeedbackFromXML(data, new_feedback)) {
            LOG_ERROR(MODULE_NAME, "Failed to parse XML feedback.");
            throw std::runtime_error("XML parsing error in feedback");
        }
        last_received_feedback_ = new_feedback;
        LOG_DEBUG(MODULE_NAME, "State deserialized successfully.");
        return last_received_feedback_;
    } catch (const std::exception& e) {
        LOG_ERROR(MODULE_NAME, "Failed to read state: " + std::string(e.what()));
        // Consider setting connected_ = false or re-throwing a more specific exception
        // For now, return the last good feedback but mark it as an error state internally
        last_received_feedback_.rt_state = RTState::Error; // Indicate problem with last read
        // Or, if we want to signal total communication loss:
        // connected_ = false; // This is a strong reaction.
        throw; // Re-throw for the caller to handle
    }
}


void UDPMotionInterface::emergencyStop() {
    std::lock_guard<std::mutex> lock(interface_mutex_);
    LOG_CRITICAL(MODULE_NAME, "Emergency Stop command initiated!");
    // This MUST send a specific E-Stop command packet to the robot.
    // The format of this packet is highly protocol-dependent.
    // It might not be a standard RobotCommandFrame.
    // For this example, we'll try to send a command that indicates E-Stop.
    // A real E-Stop might be a different UDP message or even a hardware line.

    // Example: Construct a special command or use a dedicated E-Stop packet
    // This is a placeholder for the actual E-Stop mechanism
    RobotCommandFrame estop_cmd;
    // Populate estop_cmd to signify E-Stop if the protocol uses general command frames for it
    // e.g., setting all speeds to zero, and a special flag.
    // More likely, E-Stop is a very distinct, simple packet.
    // For now, let's assume we just log it and change local state.
    // In a real system, you'd send the E-STOP signal here.

    std::vector<char> estop_buffer;
    // CreateEStopPacket(estop_buffer); // Hypothetical function
    // transport_->send(estop_buffer);

    LOG_CRITICAL(MODULE_NAME, "E-STOP signal sent (simulated). Robot should stop immediately.");
    // Update local state to reflect E-Stop
    last_received_feedback_.rt_state = RTState::Error; // Or EStop if we add it to RTState
    // Potentially set connected_ to false or expect connection to drop if E-Stop causes robot to reset comms
}


std::string UDPMotionInterface::getInterfaceName() const {
    return "UDPMotionInterface (XML)";
}

// XML serialization for RobotCommandFrame
void UDPMotionInterface::serializeCommandToXML(const RobotCommandFrame& cmd, std::vector<char>& out_buffer) {
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement* root = doc.NewElement("RobotCommand");
    doc.InsertFirstChild(root);

    // Timestamp
    auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                      std::chrono::system_clock::now().time_since_epoch())
                      .count();
    root->SetAttribute("timestamp_ms", static_cast<int64_t>(now_ms));
    root->SetAttribute("speed_ratio", cmd.speed_ratio);
    root->SetAttribute("accel_ratio", cmd.acceleration_ratio);

    // // Cartesian Target
    // tinyxml2::XMLElement* cart_target_elem = doc.NewElement("CartesianTarget");
    // cart_target_elem->SetAttribute("x",  cmd.cartesian_target.x.value());
    // cart_target_elem->SetAttribute("y",  cmd.cartesian_target.y.value());
    // cart_target_elem->SetAttribute("z",  cmd.cartesian_target.z.value());
    // cart_target_elem->SetAttribute("rx", cmd.cartesian_target.rx.value());
    // cart_target_elem->SetAttribute("ry", cmd.cartesian_target.ry.value());
    // cart_target_elem->SetAttribute("rz", cmd.cartesian_target.rz.value());
    // root->InsertEndChild(cart_target_elem);

    // Joint Target
    addAxisSetToXML(doc, root, cmd.joint_target, "JointTarget");

    tinyxml2::XMLPrinter printer;
    doc.Print(&printer);
    std::string xml_str = printer.CStr();
    out_buffer.assign(xml_str.begin(), xml_str.end());
}

// XML deserialization for RobotFeedbackFrame
bool UDPMotionInterface::deserializeFeedbackFromXML(const std::vector<char>& in_buffer, RobotFeedbackFrame& out_frame) {
    tinyxml2::XMLDocument doc;
    if (doc.Parse(in_buffer.data(), in_buffer.size()) != tinyxml2::XML_SUCCESS) {
        LOG_ERROR(MODULE_NAME, "XML Parse error: " + std::string(doc.ErrorStr()));
        return false;
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("RobotFeedback");
    if (!root) {
        LOG_ERROR(MODULE_NAME, "Missing <RobotFeedback> root element.");
        return false;
    }

    // Timestamp and other root attributes
    int64_t ts_ms = 0;
    root->QueryInt64Attribute("timestamp_ms", &ts_ms);
    // out_frame.arrival_time can be set from this, or use transport receive time.
    // For now, let transport receive time be implicit in Timed<T> wrapper if used at higher level.
    // Or, out_frame.arrival_time = std::chrono::steady_clock::now(); // Time of parsing

    root->QueryDoubleAttribute("current_speed_ratio", &out_frame.current_speed_ratio);
    int rt_state_val = static_cast<int>(RTState::Idle);
    root->QueryIntAttribute("rt_state", &rt_state_val);
    if (rt_state_val >= static_cast<int>(RTState::Idle) && rt_state_val <= static_cast<int>(RTState::Error)) { // Basic bounds check
        out_frame.rt_state = static_cast<RTState>(rt_state_val);
    }
    root->QueryBoolAttribute("target_reached", &out_frame.target_reached);
    double path_dev_val = 0.0;
    root->QueryDoubleAttribute("path_deviation_m", &path_dev_val);
    out_frame.path_deviation = Meters(path_dev_val);


    // // Cartesian Actual
    // tinyxml2::XMLElement* cart_actual_elem = root->FirstChildElement("CartesianActual");
    // if (cart_actual_elem) {
    //     double val;
    //     if (cart_actual_elem->QueryDoubleAttribute("x", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.x = Meters(val);
    //     if (cart_actual_elem->QueryDoubleAttribute("y", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.y = Meters(val);
    //     if (cart_actual_elem->QueryDoubleAttribute("z", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.z = Meters(val);
    //     if (cart_actual_elem->QueryDoubleAttribute("rx", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.rx = Radians(val);
    //     if (cart_actual_elem->QueryDoubleAttribute("ry", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.ry = Radians(val);
    //     if (cart_actual_elem->QueryDoubleAttribute("rz", &val) == tinyxml2::XML_SUCCESS) out_frame.cartesian_actual.rz = Radians(val);
    // } else { LOG_WARN(MODULE_NAME, "Missing <CartesianActual> in feedback."); }

    // Joint Actual
    if (!parseAxisSetFromXML(root, out_frame.joint_actual, "JointActual")) {
        LOG_WARN(MODULE_NAME, "Could not fully parse <JointActual> in feedback.");
    }

    return true;
}


void UDPMotionInterface::addAxisSetToXML(tinyxml2::XMLDocument& doc, tinyxml2::XMLElement* parent, const AxisSet& axes, const std::string& setName) {
    tinyxml2::XMLElement* set_elem = doc.NewElement(setName.c_str());
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        const auto& axis_data = axes.at(i); // Use .at() for const access
        AxisId aid = static_cast<AxisId>(i);
        tinyxml2::XMLElement* axis_elem = doc.NewElement(RDT::to_string(aid).c_str()); // Use RDT::to_string

        axis_elem->SetAttribute("angle_rad", axis_data.angle.value());
        axis_elem->SetAttribute("vel_rad_s", axis_data.velocity.value());
        axis_elem->SetAttribute("acc_rad_s2", axis_data.acceleration.value());
        axis_elem->SetAttribute("torque_nm", axis_data.torque); // Assuming torque is double Nm for now
        axis_elem->SetAttribute("brake", axis_data.brake_engaged);
        axis_elem->SetAttribute("servo", axis_data.servo_enabled);
        set_elem->InsertEndChild(axis_elem);
    }
    parent->InsertEndChild(set_elem);
}

bool UDPMotionInterface::parseAxisSetFromXML(tinyxml2::XMLElement* parent, AxisSet& out_axes, const std::string& setName) {
    tinyxml2::XMLElement* set_elem = parent->FirstChildElement(setName.c_str());
    if (!set_elem) {
        LOG_WARN(MODULE_NAME, "Missing <" + setName + "> element in XML.");
        return false; // Element not found is not a fatal parsing error for the whole doc, but this set is missing
    }
    bool all_found = true;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        AxisId aid = static_cast<AxisId>(i);
        tinyxml2::XMLElement* axis_elem = set_elem->FirstChildElement(RDT::to_string(aid).c_str());
        if (!axis_elem) {
            LOG_WARN(MODULE_NAME, "Missing axis <" + RDT::to_string(aid) + "> in <" + setName + ">.");
            all_found = false;
            continue; // Skip this axis, try to parse others
        }
        double d_val;
        if (axis_elem->QueryDoubleAttribute("angle_rad", &d_val) == tinyxml2::XML_SUCCESS) out_axes.at(i).angle = Radians(d_val);
        if (axis_elem->QueryDoubleAttribute("vel_rad_s", &d_val) == tinyxml2::XML_SUCCESS) out_axes.at(i).velocity = RadiansPerSecond(d_val);
        if (axis_elem->QueryDoubleAttribute("acc_rad_s2", &d_val) == tinyxml2::XML_SUCCESS) out_axes.at(i).acceleration = RadiansPerSecondSq(d_val);
        axis_elem->QueryDoubleAttribute("torque_nm", &out_axes.at(i).torque);
        axis_elem->QueryBoolAttribute("brake", &out_axes.at(i).brake_engaged);
        axis_elem->QueryBoolAttribute("servo", &out_axes.at(i).servo_enabled);
    }
    return all_found; // Returns true if the setName element was found, even if some axes were missing
}


} // namespace RDT