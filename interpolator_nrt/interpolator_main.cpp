// interpolator_main.cpp
#include "TrajectoryInterpolator.h"
// MotionProfile.h is included by TrajectoryInterpolator.h
#include "DataTypes.h"
#include "Units.h"     //    RDT::literals
#include "Logger.h"
#include "KdlKinematicSolver_mock.h" //  mock

#include <iostream>
#include <vector>
#include <memory>
#include <iomanip>   // For std::fixed, std::setprecision
#include <optional>  //  std::optional  KdlKinematicSolver_mock
#include <chrono>    //  std::chrono_literals

// MODIFICATION:  using    
using namespace RDT; 
using namespace RDT::literals; 
using namespace std::chrono_literals;


// Helper function to print TrajectoryPoint (using RDT types)
void printTrajectoryPointDetailed(const RDT::TrajectoryPoint& tp, const std::string& prefix = "", std::shared_ptr<KdlKinematicSolver_mock> solver_ptr = nullptr) {
    std::cout << prefix << std::fixed << std::setprecision(4);
    std::cout << "TP Info -- MotionType: " << static_cast<int>(tp.header.motion_type)
              << ", DataType: " << static_cast<int>(tp.header.data_type)
              << ", Speed: " << tp.command.speed_ratio * 100.0 << "%"
              << ", Blend: " << (tp.header.use_blending ? "Y" : "N")
              << ", Reached: " << (tp.header.is_target_reached_for_this_point ? "Y" : "N")
              << ", Error: " << (tp.header.has_error_at_this_point ? "Y" : "N")
              << std::endl;

    std::cout << prefix << "  Cmd Joint: " << tp.command.joint_target.toJointPoseString(2) << std::endl;
    std::cout << prefix << "  Cmd Cart:  " << tp.command.cartesian_target.toDescriptiveString() << std::endl;

    if (solver_ptr) { //   ,  FK   
        auto fk_from_cmd_joint = solver_ptr->solveFK(tp.command.joint_target);
        if (fk_from_cmd_joint) {
            std::cout << prefix << "  Cmd Joint FK: " << fk_from_cmd_joint->toDescriptiveString() << std::endl;
        }
    }
    //    Feedback ,   
}


int main() {
    Logger::setLogLevel(LogLevel::Debug);
    LOG_INFO("InterpolatorTest", "--- TrajectoryInterpolator Test (with LIN & JOINT) ---");

    auto solver = std::make_shared<KdlKinematicSolver_mock>();

    std::vector<TrajectoryPoint> user_program_waypoints;

    // P0:   (,   )
    TrajectoryPoint p0;
    p0.header.motion_type = MotionType::PTP; //    PTP
    p0.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    std::array<Radians, ROBOT_AXES_COUNT> p0_angles = {
        (0.0_deg).toRadians(), (-90.0_deg).toRadians(), (90.0_deg).toRadians(),
        (0.0_deg).toRadians(), (90.0_deg).toRadians(), (0.0_deg).toRadians()};
    p0.command.joint_target.fromAngleArray(p0_angles);
    p0.command.cartesian_target = solver->solveFK(p0.command.joint_target).value_or(CartPose{});
    p0.header.tool = ToolFrame(CartPose{0.0_m, 0.0_m, 0.15_m}, "Tool150mmZ"); // transform, name
    p0.header.base = BaseFrame(CartPose{}, "BaseWorld");
    p0.command.speed_ratio = 0.7; // 70% for subsequent PTP if it were a target

    user_program_waypoints.push_back(p0);
    LOG_INFO("InterpolatorTest", "P0 (Program Start Point):");
    printTrajectoryPointDetailed(p0, "  ", solver);

    // P1:    LIN 
    TrajectoryPoint p1_lin_target;
    p1_lin_target.header.motion_type = MotionType::LIN;
    p1_lin_target.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    p1_lin_target.command.cartesian_target = {0.5_m, 0.1_m, 0.4_m, (0.0_deg).toRadians(), (10.0_deg).toRadians(), (0.0_deg).toRadians()};
    p1_lin_target.command.speed_ratio = 0.5; // 50% speed for this LIN segment
    p1_lin_target.command.acceleration_ratio = 0.5; // 50% accel for this LIN segment
    p1_lin_target.header.tool = p0.header.tool; // Inherit tool/base
    p1_lin_target.header.base = p0.header.base;
    //  ,      IK
    p1_lin_target.command.joint_target = solver->solveIK(p1_lin_target.command.cartesian_target, p0.command.joint_target).value_or(p0.command.joint_target);

    user_program_waypoints.push_back(p1_lin_target);
    LOG_INFO("InterpolatorTest", "P1 (LIN Target):");
    printTrajectoryPointDetailed(p1_lin_target, "  ", solver);

    // P2:    JOINT  (PTP-like)
    TrajectoryPoint p2_joint_target;
    p2_joint_target.header.motion_type = MotionType::JOINT; //  PTP,   
    p2_joint_target.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    std::array<Radians, ROBOT_AXES_COUNT> p2_angles = {
        (30.0_deg).toRadians(), (-45.0_deg).toRadians(), (60.0_deg).toRadians(),
        (10.0_deg).toRadians(), (75.0_deg).toRadians(), (15.0_deg).toRadians()};
    p2_joint_target.command.joint_target.fromAngleArray(p2_angles);
    p2_joint_target.command.cartesian_target = solver->solveFK(p2_joint_target.command.joint_target).value_or(CartPose{});
    p2_joint_target.command.speed_ratio = 0.6; // 60% speed for this JOINT segment
    p2_joint_target.header.tool = p0.header.tool;
    p2_joint_target.header.base = p0.header.base;

    user_program_waypoints.push_back(p2_joint_target);
    LOG_INFO("InterpolatorTest", "P2 (JOINT Target):");
    printTrajectoryPointDetailed(p2_joint_target, "  ", solver);

    // P3:   LIN      ( P2)
    TrajectoryPoint p3_lin_orient_target;
    p3_lin_orient_target.header.motion_type = MotionType::LIN;
    p3_lin_orient_target.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    p3_lin_orient_target.command.cartesian_target = p2_joint_target.command.cartesian_target; //   XYZ ,   P2
    p3_lin_orient_target.command.cartesian_target.rz = (p2_joint_target.command.cartesian_target.rz + (90.0_deg).toRadians()).normalized(); //  Rz  90 
    p3_lin_orient_target.command.speed_ratio = 0.3; // 30% speed for orientation change
    p3_lin_orient_target.header.tool = p0.header.tool;
    p3_lin_orient_target.header.base = p0.header.base;
    p3_lin_orient_target.command.joint_target = solver->solveIK(p3_lin_orient_target.command.cartesian_target, p2_joint_target.command.joint_target).value_or(p2_joint_target.command.joint_target);

    user_program_waypoints.push_back(p3_lin_orient_target);
    LOG_INFO("InterpolatorTest", "P3 (LIN Orientation Change Target):");
    printTrajectoryPointDetailed(p3_lin_orient_target, "  ", solver);


    // ---    ---
    TrajectoryPoint current_robot_state_tp = p0; //    P0

    TrajectoryInterpolator interpolator;
    Seconds control_cycle_dt = 0.02_s; // 20 ms  (50 Hz)
    bool error_occurred = false;
    int total_interpolated_points = 0;

    for (std::size_t segment_idx = 0; segment_idx < user_program_waypoints.size() - 1; ++segment_idx) {
        const TrajectoryPoint& segment_start_for_load = current_robot_state_tp; //    
        const TrajectoryPoint& segment_target_for_load = user_program_waypoints[segment_idx + 1];

        LOG_INFO_F("InterpolatorTest", "\n--- Loading Segment %zu (Programmed MotionType: %d) ---", segment_idx + 1, static_cast<int>(segment_target_for_load.header.motion_type));
        // printTrajectoryPointDetailed(segment_start_for_load, "  Start for Load: ", solver);
        // printTrajectoryPointDetailed(segment_target_for_load, "  Target for Load: ", solver);

        try {
            interpolator.loadSegment(segment_start_for_load, segment_target_for_load);
        } catch (const std::exception& e) {
            LOG_CRITICAL_F("InterpolatorTest", "Error loading segment %zu: %s", segment_idx + 1, e.what());
            error_occurred = true;
            break;
        }

        LOG_INFO_F("InterpolatorTest", "Segment %zu loaded. Interpolator Profile Type: %d, Est. Duration: %s. Interpolating...",
                   segment_idx + 1, static_cast<int>(interpolator.getCurrentProfileDuration() > 0.0_s ? segment_target_for_load.header.motion_type : MotionType::JOINT /* fallback if duration 0 */), // This shows the type passed to interpolator
                   interpolator.getCurrentProfileDuration().toString().c_str());

        int points_in_this_segment = 0;
        while (!interpolator.isIdle() && !error_occurred) {
            TrajectoryPoint interpolated_tp;
            try {
                interpolated_tp = interpolator.nextPoint(control_cycle_dt);
            } catch (const std::exception& e) {
                LOG_ERROR_F("InterpolatorTest", "Error in nextPoint for segment %zu at t=~%s: %s",
                            segment_idx + 1, interpolator.getCurrentTimeInProfile().toString().c_str(), e.what());
                error_occurred = true;
                break;
            }
            points_in_this_segment++;
            total_interpolated_points++;

            // ---        ---
            // `interpolated_tp.command`  ,     MotionManager
            current_robot_state_tp.header = interpolated_tp.header; //  
            current_robot_state_tp.command = interpolated_tp.command; //  "" 

            //      
            current_robot_state_tp.feedback.rt_state = RTState::Moving;
            current_robot_state_tp.feedback.current_speed_ratio = interpolated_tp.command.speed_ratio;
            current_robot_state_tp.feedback.target_reached = interpolated_tp.header.is_target_reached_for_this_point;
            
            if (interpolated_tp.header.motion_type == MotionType::JOINT || interpolated_tp.header.motion_type == MotionType::PTP) {
                current_robot_state_tp.feedback.joint_actual = interpolated_tp.command.joint_target;
                current_robot_state_tp.feedback.cartesian_actual = solver->solveFK(interpolated_tp.command.joint_target).value_or(current_robot_state_tp.feedback.cartesian_actual);
            } else if (interpolated_tp.header.motion_type == MotionType::LIN) {
                current_robot_state_tp.feedback.cartesian_actual = interpolated_tp.command.cartesian_target;
                current_robot_state_tp.feedback.joint_actual = solver->solveIK(
                    interpolated_tp.command.cartesian_target,
                    current_robot_state_tp.feedback.joint_actual // Seed from last feedback
                ).value_or(current_robot_state_tp.feedback.joint_actual); // Keep last on IK fail
            }

            //   10-       
            if (points_in_this_segment % 10 == 0 || interpolator.isIdle()) {
                 LOG_DEBUG_F("InterpolatorTest", "  Seg %zu, t=%.3fs (%.1f%%), Type=%d, J1 Cmd: %.2f deg, X Cmd: %.4f m, Reached: %s",
                    segment_idx + 1,
                    interpolator.getCurrentTimeInProfile().value(),
                    (interpolator.getCurrentProfileDuration().value() > 0 ? interpolator.getCurrentTimeInProfile().value() / interpolator.getCurrentProfileDuration().value() * 100.0 : 100.0),
                    static_cast<int>(interpolated_tp.header.motion_type),
                    interpolated_tp.command.joint_target[AxisId::A1].angle.toDegrees().value(),
                    interpolated_tp.command.cartesian_target.x.value(),
                    (interpolated_tp.header.is_target_reached_for_this_point ? "Y" : "N") );
            }
        } // end while !isIdle for current segment

        if (error_occurred) break;

        // ,  current_robot_state_tp     
        current_robot_state_tp = segment_target_for_load; //      
        //  FK/IK   feedback   current_robot_state_tp
        if (current_robot_state_tp.header.motion_type == MotionType::JOINT || current_robot_state_tp.header.motion_type == MotionType::PTP) {
            current_robot_state_tp.feedback.joint_actual = current_robot_state_tp.command.joint_target;
            current_robot_state_tp.feedback.cartesian_actual = solver->solveFK(current_robot_state_tp.command.joint_target).value_or(current_robot_state_tp.feedback.cartesian_actual);
        } else if (current_robot_state_tp.header.motion_type == MotionType::LIN) {
            current_robot_state_tp.feedback.cartesian_actual = current_robot_state_tp.command.cartesian_target;
            current_robot_state_tp.feedback.joint_actual = solver->solveIK(current_robot_state_tp.command.cartesian_target, current_robot_state_tp.command.joint_target /*seed with target joints*/).value_or(current_robot_state_tp.feedback.joint_actual);
        }
        current_robot_state_tp.header.is_target_reached_for_this_point = true; //  
        current_robot_state_tp.feedback.target_reached = true;
        current_robot_state_tp.feedback.rt_state = RTState::Idle; //   

        LOG_INFO_F("InterpolatorTest", "Segment %zu finished processing. Total points in segment: %d", segment_idx + 1, points_in_this_segment);
        printTrajectoryPointDetailed(current_robot_state_tp, "  Final Robot State After Seg: ", solver);
    } // end for each segment

    LOG_INFO_F("InterpolatorTest", "Total interpolated points generated: %d", total_interpolated_points);
    if (error_occurred) {
        LOG_ERROR("InterpolatorTest", "\n--- Interpolation stopped due to an error ---");
    } else {
        LOG_INFO("InterpolatorTest", "\n--- Entire program interpolation complete ---");
    }
    return 0;
}