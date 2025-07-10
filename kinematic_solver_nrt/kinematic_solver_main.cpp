// main.cpp (was test_kinematics.cpp)
#include "KdlKinematicSolver.h" // RDT::KdlKinematicSolver
#include "KinematicModel.h"     // RDT::KinematicModel
#include "DataTypes.h"          // RDT::CartPose, RDT::AxisSet
#include "Units.h"              // RDT::Radians, RDT::Meters, RDT::literals, RDT::UnitConstants
#include "Logger.h"             // RDT::Logger

#include <iostream>
#include <memory>    // For std::unique_ptr if used, though not directly here
#include <cmath>     // For std::abs
#include <iomanip>   // For std::fixed, std::setprecision
#include <optional>  // For std::optional from solveIK

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    RDT::Logger::setLogLevel(RDT::LogLevel::Debug); // Set log level for detailed output

    LOG_INFO("KinematicsTest", "--- KDL Kinematics FK/IK Consistency Test ---");

    // For convenience in this main
    using namespace RDT;
    using namespace RDT::literals;

    try {
        // 1. Load KinematicModel for KUKA KR6 R900
        LOG_INFO("KinematicsTest", "Creating KinematicModel...");
        KinematicModel model = KinematicModel::createKR6R900();
        KdlKinematicSolver solver(model);

        const std::size_t dof = model.getChain().getNrOfJoints();
        LOG_INFO_F("KinematicsTest", "Model DoF: %zu", dof);

        // 2. Get home position as the initial test point
        AxisSet home_joints_rdt = solver.getHomePosition(); // Uses RDT::AxisSet
        LOG_INFO("KinematicsTest", "Initial Home Position (from solver):");
        std::cout << "  " << home_joints_rdt.toJointPoseString(2) << std::endl;


        // 3. Forward Kinematics (FK) from home position
        LOG_INFO("KinematicsTest", "Calculating FK from home position...");
        CartPose target_cart_pose_rdt = solver.solveFK(home_joints_rdt);
        LOG_INFO("KinematicsTest", "FK Result (Target Cartesian Pose):");
        std::cout << "  " << target_cart_pose_rdt.toDescriptiveString() << std::endl;


        // 4. Inverse Kinematics (IK) using the FK result as target, and home as seed
        LOG_INFO("KinematicsTest", "Calculating IK for the FK-derived Cartesian Pose (seed: home)...");
        std::optional<AxisSet> ik_result_opt = solver.solveIK(target_cart_pose_rdt, home_joints_rdt);

        if (!ik_result_opt) {
            LOG_ERROR("KinematicsTest", "IK FAILED: Target pose might be unreachable or near a singularity from the seed.");
            return 1;
        }
        AxisSet ik_solved_joints_rdt = *ik_result_opt;
        LOG_INFO("KinematicsTest", "IK Success. Solved Joint Positions:");
        std::cout << "  " << ik_solved_joints_rdt.toJointPoseString(2) << " (degrees)" << std::endl;


        // 5. Verify by calculating FK from the IK result
        LOG_INFO("KinematicsTest", "Calculating FK from IK result to verify consistency...");
        CartPose checked_cart_pose_rdt = solver.solveFK(ik_solved_joints_rdt);
        LOG_INFO("KinematicsTest", "FK(IK) Result (Checked Cartesian Pose):");
        std::cout << "  " << checked_cart_pose_rdt.toDescriptiveString() << std::endl;


        // 6. Compare the original FK target pose with the FK(IK) checked pose
        LOG_INFO("KinematicsTest", "Comparing original FK target with FK(IK) result...");
        Meters dx = (checked_cart_pose_rdt.x - target_cart_pose_rdt.x).abs();
        Meters dy = (checked_cart_pose_rdt.y - target_cart_pose_rdt.y).abs();
        Meters dz = (checked_cart_pose_rdt.z - target_cart_pose_rdt.z).abs();
        // Sum of absolute differences in meters
        Meters positional_error_m = dx + dy + dz;

        // Angular error (simple sum of absolute differences of Euler angles - more robust would be quaternion diff)
        Radians drx = (checked_cart_pose_rdt.rx - target_cart_pose_rdt.rx).abs();
        Radians dry = (checked_cart_pose_rdt.ry - target_cart_pose_rdt.ry).abs();
        Radians drz = (checked_cart_pose_rdt.rz - target_cart_pose_rdt.rz).abs();
        Radians angular_error_rad_sum = drx + dry + drz;

        std::cout << std::fixed << std::setprecision(6); // Higher precision for errors
        LOG_INFO_F("KinematicsTest", "Positional Error (sum of abs diff X,Y,Z): %s", positional_error_m.toString(6).c_str());
        LOG_INFO_F("KinematicsTest", "Angular Error (sum of abs diff Rx,Ry,Rz): %s (%s)",
                   angular_error_rad_sum.toString(6).c_str(),
                   angular_error_rad_sum.toDegrees().toString(4).c_str());

        // Define acceptable tolerances (these are examples, adjust as needed)
        const Meters max_acceptable_pos_error = 0.0001_m; // 0.1 mm
        const Radians max_acceptable_ang_error_sum = (0.1_deg).toRadians(); // 0.1 degree total

        if (positional_error_m <= max_acceptable_pos_error && angular_error_rad_sum <= max_acceptable_ang_error_sum) {
            LOG_INFO("KinematicsTest", "SUCCESS: FK/IK consistency check passed within tolerance.");
        } else {
            LOG_ERROR("KinematicsTest", "FAILURE: FK/IK consistency check FAILED. Errors exceed tolerance.");
            if (positional_error_m > max_acceptable_pos_error) {
                 LOG_ERROR_F("KinematicsTest", "  Positional error %s exceeds tolerance %s", positional_error_m.toString(6).c_str(), max_acceptable_pos_error.toString(6).c_str());
            }
            if (angular_error_rad_sum > max_acceptable_ang_error_sum) {
                 LOG_ERROR_F("KinematicsTest", "  Angular error %s (%s) exceeds tolerance %s (%s)",
                    angular_error_rad_sum.toString(6).c_str(), angular_error_rad_sum.toDegrees().toString(4).c_str(),
                    max_acceptable_ang_error_sum.toString(6).c_str(), max_acceptable_ang_error_sum.toDegrees().toString(4).c_str());
            }
            return 1; // Indicate failure
        }

    } catch (const std::exception& e) {
        LOG_CRITICAL_F("KinematicsTest", "An unhandled exception occurred: %s", e.what());
        return 1;
    }

    LOG_INFO("KinematicsTest", "Kinematics test completed.");
    return 0;
}