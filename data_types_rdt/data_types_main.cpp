// main.cpp
#include "DataTypes.h"
#include <iostream>
#include <vector>
#include <stdexcept>
#include <chrono>
#include <array> // For std::array in tests

// Helper ostream operators (same as before)
std::ostream& operator<<(std::ostream& os, RDT::RobotMode mode) { /* ... */ 
    switch (mode) {
        case RDT::RobotMode::Idle: os << "Idle"; break; case RDT::RobotMode::Running: os << "Running"; break;
        case RDT::RobotMode::Paused: os << "Paused"; break; case RDT::RobotMode::EStop: os << "EStop"; break;
        case RDT::RobotMode::Error: os << "Error"; break; case RDT::RobotMode::Initializing: os << "Initializing"; break;
        case RDT::RobotMode::Homing: os << "Homing"; break; case RDT::RobotMode::Jogging: os << "Jogging"; break;
        default: os << "Unknown RobotMode"; break;
    } return os;
}
std::ostream& operator<<(std::ostream& os, RDT::RTState state) { /* ... */
    switch (state) {
        case RDT::RTState::Idle: os << "Idle"; break; case RDT::RTState::Initializing: os << "Initializing"; break;
        case RDT::RTState::Moving: os << "Moving"; break; case RDT::RTState::Paused: os << "Paused"; break;
        case RDT::RTState::Error: os << "Error"; break; default: os << "Unknown RTState"; break;
    } return os;
}


void testUnitsOperations() {
    std::cout << "\n--- Testing Units Operations ---" << std::endl;
    using namespace RDT;
    using namespace RDT::literals;

    Seconds t = 2.0_s;
    MetersPerSecond v_m_s = 1.5_m_s;
    RadiansPerSecond v_rad_s = 0.5_rad_s;

    Meters dist = v_m_s * t;
    Radians angle_change = v_rad_s * t;

    std::cout << v_m_s << " * " << t << " = " << dist << std::endl;
    std::cout << v_rad_s << " * " << t << " = " << angle_change << " (" << angle_change.toDegrees() << ")" << std::endl;

    MetersPerSecond v_calc = 10.0_m / 4.0_s;
    std::cout << "10.0_m / 4.0_s = " << v_calc << std::endl;

    // Example: Radians r = 1.0_m / 1.0_s; // COMPILE ERROR - good!
}


void testAxisAndAxisSetStrict() {
    std::cout << "\n--- Testing Axis & AxisSet (Strict Units) ---" << std::endl;
    using namespace RDT;
    using namespace RDT::literals;

    std::array<Radians, ROBOT_AXES_COUNT> initial_angles = {
        (90.0_deg).toRadians(), (45.0_deg).toRadians(), 0.1_rad,
        -(30.0_deg).toRadians(), 0.0_rad, 1.0_rad
    };
    AxisSet joint_set(initial_angles);

    joint_set[AxisId::A1].velocity = 0.5_rad_s;
    joint_set[AxisId::A1].acceleration = 0.1_rad_s2;
    joint_set[AxisId::A1].servo_enabled = true;
    joint_set[AxisId::A3].brake_engaged = true;


    std::cout << "Axis A1: " << joint_set[AxisId::A1].toDescriptiveString() << std::endl;
    std::cout << "Full AxisSet (degrees):\n" << joint_set.toJointPoseString() << std::endl;

    std::array<Radians, ROBOT_AXES_COUNT> current_angles_rad = joint_set.toAngleArray();
    std::cout << "\nCurrent Angles Array (radians): ";
    for(const auto& angle_rad : current_angles_rad) {
        std::cout << angle_rad.value() << " ";
    }
    std::cout << std::endl;

    std::array<Radians, ROBOT_AXES_COUNT> new_angles_rad = {
        0.1_rad, 0.2_rad, 0.3_rad, 0.4_rad, 0.5_rad, 0.6_rad
    };
    joint_set.fromAngleArray(new_angles_rad);
    std::cout << "\nAxisSet after fromAngleArray (degrees):\n" << joint_set.toJointPoseString(2) << std::endl;

    // Test initialization from raw doubles (SHOULD NOT BE ALLOWED for angles)
    // double raw_doubles[] = {0.1,0.2,0.3,0.4,0.5,0.6};
    // std::array<double, ROBOT_AXES_COUNT> raw_array;
    // std::copy(std::begin(raw_doubles), std::end(raw_doubles), raw_array.begin());
    // AxisSet js_from_double_compile_error(raw_array); // This should now be a compile error
}

// testCartPose, testFramesAndTrajectory, testTimedStruct - similar to previous but ensure usage of new types
// ... (       ,      ._m, ._rad  ..,
//   ,          )

void testCartPoseStrict() {
    std::cout << "\n--- Testing CartPose (Strict Units) ---" << std::endl;
    using namespace RDT;
    using namespace RDT::literals;
    CartPose p1;
    p1.x = 1.0_m;
    p1.y = Millimeters(2500.0).toMeters();
    p1.z = -0.5_m;
    p1.rx = (10.0_deg).toRadians();
    p1.ry = Radians(0.2);
    p1.rz = (0.3_rad).normalized(); // Example of using normalized
    std::cout << "Pose p1: " << p1.toDescriptiveString() << std::endl;

    std::cout << "p1.x: " << p1.x << ", p1.rx (deg): " << p1.rx.toDegrees() << std::endl;

    // Using set_value_at (accessing raw double values)
    p1.set_value_at(1, 3.0); // Sets y to 3.0_m
    p1.set_value_at(4, (45.0_deg).toRadians().value()); // Sets ry value (rad)
    std::cout << "Pose p1 after set_value_at: " << p1.toDescriptiveString() << std::endl;
    std::cout << "p1.get_value_at(1) (y value): " << p1.get_value_at(1) << std::endl;
}


int main() {
    std::cout << "Robot DataTypes Test Program (Strict Units - Phase 2)" << std::endl;
    std::cout << "ROBOT_AXES_COUNT: " << RDT::ROBOT_AXES_COUNT << std::endl;

    try {
        // Test Units.h functionality (from previous main)
        using namespace RDT::literals;
        RDT::Degrees d1(90.0); RDT::Radians r1 = d1.toRadians();
        std::cout << d1 << " is " << r1 << std::endl;
        RDT::Meters m1 = 1.5_m; RDT::Millimeters mm1 = m1.toMillimeters();
        std::cout << m1 << " is " << mm1 << std::endl;
        // ----

        testUnitsOperations(); // New test for inter-unit ops
        testAxisAndAxisSetStrict();
        testCartPoseStrict(); // Renamed
        // testFramesAndTrajectory(); // Assuming it's updated for strict types
        // testTimedStruct();       // Assuming it's updated
    } catch (const std::exception& e) {
        std::cerr << "\n******\nUnhandled exception in main: " << e.what() << "\n******" << std::endl;
        return 1;
    }

    std::cout << "\nAll tests completed." << std::endl;
    return 0;
}