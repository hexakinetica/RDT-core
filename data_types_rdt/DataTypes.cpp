// DataTypes.cpp
#include "DataTypes.h"
#include <sstream>   // For std::ostringstream
#include <iomanip>   // For std::fixed, std::setprecision
#include <stdexcept> // For std::out_of_range, etc.
// No need for <vector> here unless an implementation detail requires it.

namespace RDT {

// For convenience with literals if needed inside implementations
using namespace RDT::literals;

//--------------------------------------------------------------------------
// SECTION: AxisId Helper Implementations
//--------------------------------------------------------------------------
std::string to_string(AxisId id) { /* ... same as previous correct version ... */
    switch (id) {
        case AxisId::A1: return "A1"; case AxisId::A2: return "A2"; case AxisId::A3: return "A3";
        case AxisId::A4: return "A4"; case AxisId::A5: return "A5"; case AxisId::A6: return "A6";
        default: throw std::invalid_argument("Invalid AxisId for to_string: " + std::to_string(static_cast<std::size_t>(id)));
    }
}
AxisId axisIdFromInt(int i) { /* ... same as previous correct version ... */
    if (i < 0 || i >= static_cast<int>(AxisId::Count)) {
        throw std::out_of_range("Int " + std::to_string(i) + " out of range for AxisId. Max: " + std::to_string(static_cast<int>(AxisId::Count)-1));
    }
    return static_cast<AxisId>(i);
}
int axisIdToInt(AxisId id) { /* ... same as previous correct version ... */
    if (id >= AxisId::Count) {
        throw std::out_of_range("AxisId " + std::to_string(static_cast<std::size_t>(id)) + " invalid for int conversion.");
    }
    return static_cast<int>(id);
}

//--------------------------------------------------------------------------
// SECTION: Axis Implementation
//--------------------------------------------------------------------------
std::string Axis::toDescriptiveString() const {
    std::ostringstream oss;
    oss << "Ang: " << angle.toDegrees() // Display in degrees
        << ", Vel: " << velocity.toDegreesPerSecond() // Display in deg/s
        << ", Acc: " << acceleration.value() << " rad/s^2" // TODO: Add .toDegreesPerSecondSq() if defined
        << ", Trq: " << std::fixed << std::setprecision(2) << torque << " Nm"
        << ", Brake: " << (brake_engaged ? "ON" : "OFF")
        << ", Servo: " << (servo_enabled ? "ON" : "OFF");
    return oss.str();
}

 bool Axis::operator==(const Axis& other) const {
        return angle == other.angle &&
               velocity == other.velocity &&
               acceleration == other.acceleration &&
               std::abs(torque - other.torque) < UnitConstants::DEFAULT_EPSILON && //  double
               brake_engaged == other.brake_engaged &&
               servo_enabled == other.servo_enabled;
    }
bool Axis::operator!=(const Axis& other) const { return !(*this == other); }

//--------------------------------------------------------------------------
// SECTION: AxisSet Implementation
//--------------------------------------------------------------------------
AxisSet::AxisSet() {
    // Each Axis in data_ is default-constructed
}

// STRICTLY Radians based constructor
AxisSet::AxisSet(const std::array<Radians, ROBOT_AXES_COUNT>& axis_angles) {
    if (axis_angles.size() != ROBOT_AXES_COUNT) { // Should not happen with std::array type
         throw std::invalid_argument("Input array size mismatch for AxisSet constructor.");
    }
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        data_[i].angle = axis_angles[i];
        // Other members of Axis (velocity, etc.) are default initialized.
    }
}

Axis& AxisSet::operator[](AxisId id) { return data_[axisIdToInt(id)]; }
const Axis& AxisSet::operator[](AxisId id) const { return data_[axisIdToInt(id)]; }
Axis& AxisSet::at(std::size_t idx) {
    if (idx >= ROBOT_AXES_COUNT) throw std::out_of_range("AxisSet index out of range: " + std::to_string(idx));
    return data_[idx];
}
const Axis& AxisSet::at(std::size_t idx) const {
    if (idx >= ROBOT_AXES_COUNT) throw std::out_of_range("AxisSet const index out of range: " + std::to_string(idx));
    return data_[idx];
}
std::size_t AxisSet::size() const { return ROBOT_AXES_COUNT; }

std::string AxisSet::toJointPoseString(int precision_deg) const {
    std::ostringstream oss;
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        oss << RDT::to_string(static_cast<AxisId>(i)) << "="
            << data_[i].angle.toDegrees().toString(precision_deg);
        if (i < ROBOT_AXES_COUNT - 1) oss << "; ";
    }
    return oss.str();
}

std::array<Radians, ROBOT_AXES_COUNT> AxisSet::toAngleArray() const {
    std::array<Radians, ROBOT_AXES_COUNT> out;
    out.fill(Radians{});
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        out[i] = data_[i].angle;
    }
    return out;
}

void AxisSet::fromAngleArray(const std::array<Radians, ROBOT_AXES_COUNT>& new_angles) {
    if (new_angles.size() != ROBOT_AXES_COUNT) {
         throw std::invalid_argument("Input array size mismatch for AxisSet::fromAngleArray.");
    }
    for (std::size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        data_[i].angle = new_angles[i];
    }
}

bool AxisSet::operator==(const AxisSet& other) const {
        // data_  std::array,   operator== ,   T  operator==
        return data_ == other.data_;
    }
bool AxisSet::operator!=(const AxisSet& other) const { return !(*this == other); }
    
//--------------------------------------------------------------------------
// SECTION: CartPose Implementation
//--------------------------------------------------------------------------
std::string CartPose::toDescriptiveString() const {
    std::ostringstream oss;
    oss << "CartPose(X: " << x          // RDT::Meters has operator<<
        << ", Y: " << y
        << ", Z: " << z
        << ", Rx: " << rx.toDegrees() // Display angles in degrees for readability
        << ", Ry: " << ry.toDegrees()
        << ", Rz: " << rz.toDegrees() << ")";
    return oss.str();
}

double CartPose::get_value_at(std::size_t index) const {
    switch (index) {
        case 0: return x.value(); case 1: return y.value(); case 2: return z.value();
        case 3: return rx.value();case 4: return ry.value();case 5: return rz.value();
        default: throw std::out_of_range("CartPose get_value_at index out of range: " + std::to_string(index));
    }
}
void CartPose::set_value_at(std::size_t index, double raw_value) {
    switch (index) {
        case 0: x = Meters(raw_value); break; case 1: y = Meters(raw_value); break; case 2: z = Meters(raw_value); break;
        case 3: rx = Radians(raw_value); break;case 4: ry = Radians(raw_value); break;case 5: rz = Radians(raw_value); break;
        default: throw std::out_of_range("CartPose set_value_at index out of range: " + std::to_string(index));
    }
}

bool CartPose::operator==(const CartPose& other) const {
        return x == other.x && y == other.y && z == other.z &&
               rx == other.rx && ry == other.ry && rz == other.rz;
    }
    
bool CartPose::operator!=(const CartPose& other) const { return !(*this == other); }

//--------------------------------------------------------------------------
// SECTION: Frame Types Implementation
//--------------------------------------------------------------------------
ToolFrame::ToolFrame() : name("DefaultTool"), transform() {}
ToolFrame::ToolFrame(CartPose t, std::string n) : name(std::move(n)), transform(t) { if (name.empty()) name = "DefaultTool"; }
ToolFrame::ToolFrame(std::string n, CartPose t) : name(std::move(n)), transform(t) { if (name.empty()) name = "DefaultTool"; }
bool ToolFrame::operator==(const ToolFrame& other) const {
    return name == other.name && transform.x == other.transform.x && transform.y == other.transform.y &&
           transform.z == other.transform.z && transform.rx == other.transform.rx &&
           transform.ry == other.transform.ry && transform.rz == other.transform.rz;
}
bool ToolFrame::operator!=(const ToolFrame& other) const { return !(*this == other); }

BaseFrame::BaseFrame() : name("DefaultBase"), transform() {}
BaseFrame::BaseFrame(CartPose t, std::string n) : name(std::move(n)), transform(t) { if (name.empty()) name = "DefaultBase"; }
BaseFrame::BaseFrame(std::string n, CartPose t) : name(std::move(n)), transform(t) { if (name.empty()) name = "DefaultBase"; }
bool BaseFrame::operator==(const BaseFrame& other) const {
    return name == other.name && transform.x == other.transform.x && transform.y == other.transform.y &&
           transform.z == other.transform.z && transform.rx == other.transform.rx &&
           transform.ry == other.transform.ry && transform.rz == other.transform.rz;
}
bool BaseFrame::operator!=(const BaseFrame& other) const { return !(*this == other); }

} // namespace RDT