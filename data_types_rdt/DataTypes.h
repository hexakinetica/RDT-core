// DataTypes.h
#ifndef DATATYPES_H
#define DATATYPES_H

#pragma once

#include "Units.h"  // Integrate Units.h

#include <array>
#include <string>
#include <chrono>   // For std::chrono types (like TrajectoryPointHeader::segment_duration)
#include <cstddef>  // For std::size_t
#include <cstdint>  // For uintXX_t types
#include <vector>

/**
 * @file DataTypes.h
 * @brief Declares core data types and enumerations for robot motion control within the RDT namespace.
 *        Uses strong types for physical units from Units.h.
 */
namespace RDT {

// For convenience with default member initializers using literals
using namespace RDT::literals;

//--------------------------------------------------------------------------
// SECTION: Core Enums and System State
//--------------------------------------------------------------------------
enum class RobotMode : uint8_t { /* ... same ... */ Idle, Running, Paused, EStop, Error, Initializing, Homing, Jogging };
struct SystemState { /* ... same ... */ RobotMode mode = RobotMode::Idle; std::string message; int error_code = 0; };

//--------------------------------------------------------------------------
// SECTION: Axis Related Types
//--------------------------------------------------------------------------
constexpr std::size_t ROBOT_AXES_COUNT = 6;
enum class AxisId : std::size_t { /* ... same ... */ A1 = 0, A2, A3, A4, A5, A6, Count };

struct Axis {
    Radians angle = 0.0_rad;
    RadiansPerSecond velocity = 0.0_rad_s;
    RadiansPerSecondSq acceleration = 0.0_rad_s2;
    double torque = 0.0; // Placeholder for NewtonMeters or similar
    bool brake_engaged = false;
    bool servo_enabled = false;

    [[nodiscard]] std::string toDescriptiveString() const; // Implemented in .cpp

    [[nodiscard]] bool operator==(const Axis& other) const;
    [[nodiscard]] bool operator!=(const Axis& other) const;
};

class AxisSet {
public:
    using Container = std::array<Axis, ROBOT_AXES_COUNT>;
    AxisSet();
    explicit AxisSet(const std::array<Radians, ROBOT_AXES_COUNT>& axis_angles); // STRICTLY Radians

    [[nodiscard]] Axis& operator[](AxisId id);
    [[nodiscard]] const Axis& operator[](AxisId id) const;
    [[nodiscard]] Axis& at(std::size_t idx);
    [[nodiscard]] const Axis& at(std::size_t idx) const;
    [[nodiscard]] std::size_t size() const;

    [[nodiscard]] std::string toJointPoseString(int precision_deg = 1) const;
    [[nodiscard]] std::array<Radians, ROBOT_AXES_COUNT> toAngleArray() const; // Returns array of Radians
    void fromAngleArray(const std::array<Radians, ROBOT_AXES_COUNT>& new_angles); // Takes array of Radians

    [[nodiscard]] bool operator==(const AxisSet& other) const;
    [[nodiscard]] bool operator!=(const AxisSet& other) const ;

private:
    Container data_;
};

[[nodiscard]] std::string to_string(AxisId id);
[[nodiscard]] AxisId axisIdFromInt(int i);
[[nodiscard]] int axisIdToInt(AxisId id);

//--------------------------------------------------------------------------
// SECTION: Cartesian Pose
//--------------------------------------------------------------------------
struct CartPose {
    Meters x = 0.0_m;   Meters y = 0.0_m;   Meters z = 0.0_m;
    Radians rx = 0.0_rad; Radians ry = 0.0_rad; Radians rz = 0.0_rad;

    [[nodiscard]] std::string toDescriptiveString() const;
    [[nodiscard]] double get_value_at(std::size_t index) const; // Access raw double value
    void set_value_at(std::size_t index, double raw_value);   // Set from raw double value
    
    [[nodiscard]] bool operator==(const CartPose& other) const;
    [[nodiscard]] bool operator!=(const CartPose& other) const;
};

//--------------------------------------------------------------------------
// SECTION: Utility Types
//--------------------------------------------------------------------------
template<typename T> struct Timed { /* ... same ... */
    T data;
    std::chrono::time_point<std::chrono::steady_clock> timestamp;
    Timed() = default;
    explicit Timed(T d) : data(std::move(d)), timestamp(std::chrono::steady_clock::now()) {}
    Timed(T d, std::chrono::time_point<std::chrono::steady_clock> ts) : data(std::move(d)), timestamp(ts) {}
};

//--------------------------------------------------------------------------
// SECTION: Frame Types
//--------------------------------------------------------------------------
struct ToolFrame { /* ... same, uses CartPose ... */
    std::string name = "DefaultTool"; CartPose transform{};
    ToolFrame(); explicit ToolFrame(CartPose t, std::string n = "DefaultTool");
    explicit ToolFrame(std::string n, CartPose t = {});
    [[nodiscard]] bool operator==(const ToolFrame& other) const;
    [[nodiscard]] bool operator!=(const ToolFrame& other) const;
};
struct BaseFrame { /* ... same, uses CartPose ... */
    std::string name = "DefaultBase"; CartPose transform{};
    BaseFrame(); explicit BaseFrame(CartPose t, std::string n = "DefaultBase");
    explicit BaseFrame(std::string n, CartPose t = {});
    [[nodiscard]] bool operator==(const BaseFrame& other) const;
    [[nodiscard]] bool operator!=(const BaseFrame& other) const;
};

//--------------------------------------------------------------------------
// SECTION: NEW: Detailed fault reporting structures 
//--------------------------------------------------------------------------
/**
 * @enum FaultType
 * @brief Categorizes the type of limit violation detected.
 */
enum class FaultType : uint8_t {
    None,           ///< No fault.
    VelocityLimit,  ///< A velocity limit was exceeded. This is typically a WARNING, handled internally by subdividing the move.
    PositionLimit   ///< A position limit was exceeded. This is a critical ERROR that stops motion.
};

/**
 * @struct FaultData
 * @brief Contains detailed information about a fault condition.
 */
struct FaultData {
    FaultType type = FaultType::None; ///< The type of fault.
    AxisId axis = AxisId::A1;         ///< The axis on which the fault occurred.
    
    // Values are in human-readable, type-safe units for logging and display.
    Degrees actual_position_deg{0.0};    ///< The actual/commanded position that caused the fault.
    Degrees limit_position_deg{0.0};     ///< The position limit that was exceeded.
    DegreesPerSecond actual_velocity_deg_s{0.0}; ///< The actual (calculated) velocity that caused the fault.
    DegreesPerSecond limit_velocity_deg_s{0.0};  ///< The velocity limit that was exceeded.
};


//--------------------------------------------------------------------------
// SECTION: Trajectory Related Types
//--------------------------------------------------------------------------
enum class WaypointDataType : uint8_t { /* ... same ... */ NOTYPE, CARTESIAN_DOMINANT_CMD, JOINT_DOMINANT_CMD, JOINT_DOMINANT_FB, FULL_FB };
enum class RTState : uint8_t { /* ... same ... */ Idle, Initializing, Moving, Paused, Error };
enum class MotionType : uint8_t { /* ... same ... */ JOINT, PTP, LIN, SPLINE, CIRC };

struct RobotCommandFrame {
    CartPose cartesian_target{}; AxisSet joint_target{};
    double speed_ratio = 1.0; double acceleration_ratio = 1.0; // Unitless ratios
    RobotCommandFrame() = default;
};

// *** MODIFIED: RobotFeedbackFrame includes FaultData ***
struct RobotFeedbackFrame {
    CartPose cartesian_actual{}; 
    AxisSet joint_actual{};
    double current_speed_ratio = 0.0; 
    RTState rt_state = RTState::Idle;
    bool target_reached = false; 
    std::chrono::steady_clock::time_point arrival_time{};
    Meters path_deviation = 0.0_m;
    
    FaultData fault; ///< Detailed information about any fault detected in this cycle.

    RobotFeedbackFrame() = default;
};

struct TrajectoryPointHeader {
    WaypointDataType data_type = WaypointDataType::NOTYPE; MotionType motion_type = MotionType::JOINT;
    bool use_blending = false; 
    Seconds segment_duration{0};
    ToolFrame tool{}; BaseFrame base{};
    uint64_t trajectory_id = 0;
    uint32_t sequence_index = 0;
    bool is_target_reached_for_this_point = false;
    bool has_error_at_this_point = false;
    std::string user_comment;
    TrajectoryPointHeader() = default;
};
struct TrajectoryPoint {
    TrajectoryPointHeader header{};
    RobotCommandFrame command{};
    RobotFeedbackFrame feedback{};
    TrajectoryPoint() = default;
};

//--------------------------------------------------------------------------
// SECTION: Trajectory Related Types
//--------------------------------------------------------------------------
struct RobotLimits {
    /**
     * @brief Joint position limits [min, max] for each axis.
     */
    std::array<std::pair<Degrees, Degrees>, ROBOT_AXES_COUNT> joint_position_limits_deg;

    /**
     * @brief Maximum joint velocity for each axis.
     */
    std::array<DegreesPerSecond, ROBOT_AXES_COUNT> joint_velocity_limits_deg_s;
};


} // namespace RDT
#endif // DATATYPES_H
