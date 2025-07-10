// trajectory_planner_main.cpp
#include "TrajectoryPlanner.h"      // RDT::TrajectoryPlanner
#include "DataTypes.h"            // RDT::TrajectoryPoint, AxisSet, CartPose, etc.
#include "Units.h"                // RDT::literals and unit types
#include "Logger.h"               // RDT::Logger
#include "KinematicModel.h"       // RDT::KinematicModel
#include "KdlKinematicSolver.h"   // RDT::KdlKinematicSolver
// FrameTransformer.h    TrajectoryPlanner.h  DataTypes.h (   )

#include <iostream>
#include <vector>
#include <memory>
#include <iomanip>
#include <chrono>
#include <thread>
#include <array>

//  
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;

//     TrajectoryPoint ( ,   MotionManager)
void printPlannerOutputPoint(const TrajectoryPoint& pt, int point_num, const std::string& segment_info) {
    std::cout << std::fixed << std::setprecision(4);
    //    ,     MotionManager
    // Header.tool  Header.base  pt       
    std::cout << "  " << segment_info << " OutPt." << std::setw(3) << point_num
              << " Typ:" << static_cast<int>(pt.header.motion_type)
              << " J1Cmd:" << pt.command.joint_target[AxisId::A1].angle.toDegrees()
              << " XCmdFlng:" << pt.command.cartesian_target.x //   
              << " Tool:'" << pt.header.tool.name << "' Base:'" << pt.header.base.name << "'"
              << " Reached:" << (pt.header.is_target_reached_for_this_point ? "Y" : "N")
              << std::endl;
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    Logger::setLogLevel(LogLevel::Debug);
    LOG_INFO("PlannerMain", "--- TrajectoryPlanner Test (Base Coordinates, Flange Targets) ---");

    // 1.  
    LOG_INFO("PlannerMain", "Initializing components...");
    KinematicModel kuka_model = KinematicModel::createKR6R900();
    auto solver = std::make_shared<KdlKinematicSolver>(kuka_model);
    auto interpolator = std::make_shared<TrajectoryInterpolator>();
    // FrameTransformer  ,   

    TrajectoryPlanner planner(solver, interpolator);

    // 2.    (    )
    TrajectoryPoint initial_robot_tp;
    initial_robot_tp.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    //     {0.0, 0.0, 0.2, 0.0, 1.5708, 0.0}
    std::array<Radians, ROBOT_AXES_COUNT> initial_angles = {
        0.0_rad, 0.0_rad, 0.2_rad, 0.0_rad, 1.5708_rad, 0.0_rad
    };
    initial_robot_tp.command.joint_target.fromAngleArray(initial_angles);
    // cartesian_target ()    setCurrentRobotState  FK
    initial_robot_tp.header.tool = ToolFrame(); //    
    initial_robot_tp.header.base = BaseFrame(); //    

    planner.setCurrentRobotState(initial_robot_tp);
    if (planner.hasError()) {
        LOG_CRITICAL_F("PlannerMain", "Error initializing planner's current state: %s", planner.getErrorMessage().c_str());
        return 1;
    }
    LOG_INFO("PlannerMain", "Planner initialized with current robot state.");

    // 3.   (       )
    std::vector<TrajectoryPoint> user_program_targets;

    // P1: LIN   
    TrajectoryPoint target_p1;
    target_p1.header.motion_type = MotionType::LIN;
    target_p1.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    //    ,        
    target_p1.command.cartesian_target = {0.3_m, 0.2_m, 0.4_m, 0.0_rad, 1.5708_rad, 0.0_rad};
    user_program_targets.push_back(target_p1);

    // P2: LIN    ( P1   Y)
    TrajectoryPoint target_p2 = target_p1; //   P1
    target_p2.command.cartesian_target.y = -0.2_m;
    // tool  base  
    user_program_targets.push_back(target_p2);

    // P3: LIN    (  P1,    )
    TrajectoryPoint target_p3 = target_p1; //   P1
    target_p3.command.speed_ratio = 0.5;   //  speed_percent = 50.0
    // tool  base  
    user_program_targets.push_back(target_p3);

    // P4: JOINT 
    TrajectoryPoint target_p4_joint;
    target_p4_joint.header.motion_type = MotionType::JOINT;
    target_p4_joint.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    //     {0.4, -0.2, 0.3, 0.0, 1.2, 0.0}
    std::array<Radians, ROBOT_AXES_COUNT> p4_angles = {
        0.4_rad, -0.2_rad, 0.3_rad, 0.0_rad, 1.2_rad, 0.0_rad
    };
    target_p4_joint.command.joint_target.fromAngleArray(p4_angles);
    user_program_targets.push_back(target_p4_joint);

    // P5: LIN ,   ( )
    TrajectoryPoint target_p5_ik_fail;
    target_p5_ik_fail.header.motion_type = MotionType::LIN;
    target_p5_ik_fail.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    target_p5_ik_fail.command.cartesian_target = {2.0_m, 2.0_m, 2.0_m, 0.0_rad, 0.0_rad, 0.0_rad}; // 
    user_program_targets.push_back(target_p5_ik_fail);

    // P6: LIN  (  ,  P5  )
    TrajectoryPoint target_p6;
    target_p6.header.motion_type = MotionType::LIN;
    target_p6.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    target_p6.command.cartesian_target = {0.2_m, -0.2_m, 0.5_m, 0.0_rad, 1.5708_rad, 0.0_rad};
    target_p6.command.speed_ratio = 0.5;
    user_program_targets.push_back(target_p6);


    // 4.    
    Seconds sim_dt_sample = 0.1_s;     // 10  (100 )
    Seconds sim_window_duration = 1.0_s; //   100  (10   100 )

    for (std::size_t i = 0; i < user_program_targets.size(); ++i) {
        const TrajectoryPoint& next_target_for_flange_in_base = user_program_targets[i];

        //           , Header.tool.name  Header.base.name  
        LOG_INFO_F("PlannerMain", "\n--- Processing Point %zu (Flange in Base Frame): Type=%d ---",
                   i + 1, static_cast<int>(next_target_for_flange_in_base.header.motion_type));
        LOG_INFO_F("PlannerMain", "  Target Flange Pose: %s",
                   next_target_for_flange_in_base.command.cartesian_target.toDescriptiveString().c_str());
        if(next_target_for_flange_in_base.header.motion_type == MotionType::JOINT || next_target_for_flange_in_base.header.motion_type == MotionType::PTP){
            LOG_INFO_F("PlannerMain", "  Target Joint Pose (deg): %s",
                       next_target_for_flange_in_base.command.joint_target.toJointPoseString().c_str());
        }


        if (planner.hasError()){
            LOG_ERROR_F("PlannerMain", "Planner in error state BEFORE adding target %zu. Error: %s. Stopping program.",
                        i+1, planner.getErrorMessage().c_str());
            break;
        }

        if (!planner.addTargetWaypoint(next_target_for_flange_in_base)) {
            if (planner.hasError()) {
                LOG_ERROR_F("PlannerMain", "Failed to add target waypoint %zu. Error: %s. Stopping program.",
                            i + 1, planner.getErrorMessage().c_str());
            } else {
                LOG_WARN_F("PlannerMain", "Failed to add target waypoint %zu (planner busy or state not ready). Skipping target.",
                           i + 1);
            }
            if(planner.hasError()) break;
            continue;
        }
        LOG_INFO_F("PlannerMain", "Target %zu added to planner. Interpolating segment...", i + 1);

        int total_points_in_segment = 0;
        while (!planner.isCurrentSegmentDone()) {
            if (planner.hasError()) {
                LOG_ERROR_F("PlannerMain", "Error during interpolation for target %zu. Error: %s. Stopping segment.",
                            i + 1, planner.getErrorMessage().c_str());
                break;
            }

            std::vector<TrajectoryPoint> window = planner.getNextPointWindow(sim_dt_sample, sim_window_duration);

            if (planner.hasError() && window.empty()) {
                LOG_ERROR_F("PlannerMain", "Error generating window for target %zu. Error: %s. Stopping segment.",
                            i + 1, planner.getErrorMessage().c_str());
                break;
            }
            if (window.empty() && !planner.isCurrentSegmentDone()) {
                LOG_DEBUG_F("PlannerMain", "Window for target %zu is empty, but segment not done. Continuing.", i + 1);
                std::this_thread::sleep_for(1ms); // Avoid busy loop if something is strange
                continue;
            }
            if (window.empty() && planner.isCurrentSegmentDone()){
                LOG_DEBUG_F("PlannerMain", "Window for target %zu is empty AND segment done. Finishing segment.", i + 1);
                break;
            }

            LOG_DEBUG_F("PlannerMain", "  Generated window with %zu RT points for target %zu.", window.size(), i + 1);
            for (const TrajectoryPoint& rt_point_to_mm : window) {
                total_points_in_segment++;
                // ,  MotionManager   
                //  current_simulated_robot_state    main
                //           MotionManager
                // current_simulated_robot_state.command = rt_point_to_mm.command;
                // current_simulated_robot_state.header = rt_point_to_mm.header;


                if (total_points_in_segment % 5 == 0 || rt_point_to_mm.header.is_target_reached_for_this_point) {
                    printPlannerOutputPoint(rt_point_to_mm, total_points_in_segment, "Seg" + std::to_string(i+1));
                }
            }
             //  ,   MotionManager   
            // std::this_thread::sleep_for(sim_window_duration); //  ,    getNextPointWindow
        } // end while segment not done

        if (planner.hasError()) {
            LOG_ERROR_F("PlannerMain", "Segment to target %zu FAILED processing.", i + 1);
            break; 
        } else {
            LOG_INFO_F("PlannerMain", "--- Target %zu processed successfully (%d RT points generated). ---", i + 1, total_points_in_segment);
        }
    } // end for each user target

    LOG_INFO("PlannerMain", "\n--- TrajectoryPlanner Program Test Finished ---");
    if (planner.hasError()) {
        LOG_ERROR_F("PlannerMain", "Execution stopped with planner error: %s", planner.getErrorMessage().c_str());
    } else {
        LOG_INFO("PlannerMain", "All processable targets handled.");
    }
    //      (     )
    // TrajectoryPoint final_planner_state = planner.getCurrentRobotState(); //     Planner
    // LOG_INFO_F("PlannerMain", "Final planner state (flange in base): J1=%.4f, X=%.4f",
    //           final_planner_state.command.joint_target[AxisId::A1].angle.toDegrees().value(),
    //           final_planner_state.command.cartesian_target.x.value());


    return 0;
}