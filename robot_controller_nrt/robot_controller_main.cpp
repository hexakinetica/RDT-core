// robot_controller_main.cpp
#include "RobotController.h"
#include "MotionManager.h"
#include "TrajectoryPlanner.h"
#include "KinematicModel.h"
#include "KdlKinematicSolver.h"
#include "TrajectoryInterpolator.h"
#include "FakeMotionInterface.h"
#include "StateData.h"
#include "Units.h" // ,     

#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <chrono>
#include <iomanip>
#include <vector>
#include <array>

//  
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals; //  _ms, _s   std::chrono

// Helper to print selected StateData fields (     )
void printStateDataSnapshot(const StateData& sd, const std::string& context_msg) {
    LOG_INFO_F("StateDataSnpsht", "--- StateData Snapshot: %s ---", context_msg.c_str());
    TrajectoryPoint fb_tp = sd.getFbTrajectoryPoint();
    TrajectoryPoint cmd_tp = sd.getCmdTrajectoryPoint(); 

    LOG_INFO_F("StateDataSnpsht", "  RobotMode: %d, EStop: %s, Error: %s ('%s')",
        static_cast<int>(sd.getRobotMode()), sd.isEstopActive() ? "Y" : "N",
        sd.hasActiveError() ? "Y" : "N", sd.getSystemMessage().c_str());
    LOG_INFO_F("StateDataSnpsht", "  SpeedRatio: %.0f%%", sd.getGlobalSpeedRatio() * 100.0);
    LOG_INFO_F("StateDataSnpsht", "  ActiveTool: '%s', ActiveBase: '%s'",
        sd.getActiveToolFrame().name.c_str(), sd.getActiveBaseFrame().name.c_str());
    LOG_INFO_F("StateDataSnpsht", "  Feedback TCP (in Base, Tool '%s'): %s",
        fb_tp.header.tool.name.c_str(), 
        fb_tp.feedback.cartesian_actual.toDescriptiveString().c_str());
    LOG_INFO_F("StateDataSnpsht", "  Feedback Joints (deg): %s",
        fb_tp.feedback.joint_actual.toJointPoseString().c_str());
    LOG_INFO_F("StateDataSnpsht", "  Command Target TCP (in UserFrame '%s', Tool '%s'): %s",
        cmd_tp.header.base.name.c_str(), cmd_tp.header.tool.name.c_str(),
        cmd_tp.command.cartesian_target.toDescriptiveString().c_str());
    LOG_INFO("StateDataSnpsht", "--- End Snapshot ---");
}


int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    Logger::setLogLevel(LogLevel::Debug);
    LOG_INFO("RC_Main", "--- RobotController Full Pipeline Test ---");

    // 1.  
    auto state_data = std::make_shared<StateData>();
    KinematicModel kuka_model = KinematicModel::createKR6R900();
    auto solver = std::make_shared<KdlKinematicSolver>(kuka_model);
    auto interpolator = std::make_shared<TrajectoryInterpolator>();
    auto planner = std::make_shared<TrajectoryPlanner>(solver, interpolator);

    unsigned int mm_cycle_ms = 20; // MotionManager cycle period: 20ms = 50Hz
    auto motion_interface = std::make_unique<FakeMotionInterface>();
    auto motion_manager = std::make_shared<MotionManager>(std::move(motion_interface), mm_cycle_ms);

    RobotController robot_controller(planner, motion_manager, solver, state_data);

    // 2.    ( )
    TrajectoryPoint initial_user_state_cmd;
    initial_user_state_cmd.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    std::array<Radians, ROBOT_AXES_COUNT> initial_angles = {
                0.0_rad, 0.0_rad, 0.2_rad, 0.0_rad, 1.5708_rad, 0.0_rad
    };
    initial_user_state_cmd.command.joint_target.fromAngleArray(initial_angles);
    //  Tool  Base,      "" 
    initial_user_state_cmd.header.tool = ToolFrame(); // T_flange_TCP
    initial_user_state_cmd.header.base = BaseFrame(); // T_world_RobotBase (  World=RobotBase)

    LOG_INFO("RC_Main", "Initializing RobotController...");
    if (!robot_controller.initialize(initial_user_state_cmd)) {
        LOG_CRITICAL_F("RC_Main", "RobotController initialization failed! Error: %s", state_data->getSystemMessage().c_str());
        return 1;
    }
    LOG_INFO_F("RC_Main", "RobotController initialized. Internal State: %d", static_cast<int>(robot_controller.getInternalControllerState()));
    printStateDataSnapshot(*state_data, "After Initialization");

    // 3.   (  )
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
    // TrajectoryPoint target_p5_ik_fail;
    // target_p5_ik_fail.header.motion_type = MotionType::LIN;
    // target_p5_ik_fail.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    // target_p5_ik_fail.command.cartesian_target = {2.0_m, 2.0_m, 2.0_m, 0.0_rad, 0.0_rad, 0.0_rad}; // 
    // user_program_targets.push_back(target_p5_ik_fail);

    // P6: LIN  (  ,  P5  )
    // TrajectoryPoint target_p6;
    // target_p6.header.motion_type = MotionType::LIN;
    // target_p6.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    // target_p6.command.cartesian_target = {0.2_m, -0.2_m, 0.5_m, 0.0_rad, 1.5708_rad, 0.0_rad};
    // target_p6.command.speed_ratio = 0.5;
    // user_program_targets.push_back(target_p6);


    // 4.      RobotController
    //       RobotController.h
    // static constexpr Seconds PLANNER_DT_SAMPLE = 0.01_s;
    // static constexpr Seconds PLANNER_WINDOW_DURATION = 0.10_s;


    for (std::size_t i = 0; i < user_program_targets.size(); ++i) {
        const auto& next_target_wp_user = user_program_targets[i]; //   

        LOG_INFO_F("RC_Main", "\n--- Executing to User Waypoint %zu (Type: %d, Tool: '%s', Base: '%s') ---",
                   i + 1, static_cast<int>(next_target_wp_user.header.motion_type),
                   next_target_wp_user.header.tool.name.c_str(),
                   next_target_wp_user.header.base.name.c_str());
        LOG_INFO_F("RC_Main", "  User Target TCP (in its UserFrame): %s",
                   next_target_wp_user.command.cartesian_target.toDescriptiveString().c_str());
        if(next_target_wp_user.header.data_type == WaypointDataType::JOINT_DOMINANT_CMD){
            LOG_INFO_F("RC_Main", "  User Target Joints (deg): %s",
                       next_target_wp_user.command.joint_target.toJointPoseString().c_str());
        }

        if (robot_controller.getInternalControllerState() == ControllerState::Error) {
            LOG_ERROR_F("RC_Main", "Controller in error state BEFORE processing waypoint %zu. Error: '%s'. Aborting program.",
                        i + 1, state_data->getSystemMessage().c_str());
            break;
        }

        if (!robot_controller.executeMotionToTarget(next_target_wp_user)) {
            LOG_ERROR_F("RC_Main", "Failed to initiate motion to waypoint %zu. Controller State: %d. Error: '%s'",
                        i + 1, static_cast<int>(robot_controller.getInternalControllerState()), state_data->getSystemMessage().c_str());
            if (robot_controller.getInternalControllerState() == ControllerState::Error) {
                LOG_ERROR("RC_Main", "Controller entered error state. Aborting program.");
                break;
            }
            //   ,   "",     
            LOG_WARN_F("RC_Main", "Skipping waypoint %zu as controller could not accept it now.", i + 1);
            continue;
        }

        LOG_INFO_F("RC_Main", "Motion to waypoint %zu initiated. Waiting for completion...", i + 1);
        
        while (robot_controller.isMotionTaskActive()) {
            std::this_thread::sleep_for(100ms); //   
            LOG_DEBUG_F("RC_Main", "  Waiting... RC_State: %d, MM_Queue: %zu, Planner_Idle: %s",
                static_cast<int>(robot_controller.getInternalControllerState()),
                motion_manager->getCommandQueueSize(),
                planner->isCurrentSegmentDone() ? "Y" : "N");
            
            if (robot_controller.getInternalControllerState() == ControllerState::Error) {
                LOG_ERROR_F("RC_Main", "Controller entered ERROR state during motion to waypoint %zu.", i+1);
                break; 
            }
        }
        
        printStateDataSnapshot(*state_data, "After attempting motion to WP " + std::to_string(i+1));

        if (robot_controller.getInternalControllerState() == ControllerState::Error) {
            LOG_ERROR_F("RC_Main", "Motion to waypoint %zu FAILED. Error: '%s'. Aborting program.",
                        i + 1, state_data->getSystemMessage().c_str());
            break;
        }
        LOG_INFO_F("RC_Main", "Successfully processed waypoint %zu.", i + 1);
    }

    LOG_INFO("RC_Main", "\n--- Program Execution Finished (or Aborted) ---");
    printStateDataSnapshot(*state_data, "Final State");

    LOG_INFO("RC_Main", "Requesting E-Stop...");
    robot_controller.emergencyStop();
    std::this_thread::sleep_for(300ms); 
    printStateDataSnapshot(*state_data, "After E-Stop");

    LOG_INFO("RC_Main", "Requesting Reset...");
    robot_controller.reset(); 
    std::this_thread::sleep_for(300ms);
    printStateDataSnapshot(*state_data, "After Reset");

    LOG_INFO("RC_Main", "Stopping MotionManager and RobotController (via RC destructor)...");
    motion_manager->stop(); //   MotionManager

    LOG_INFO("RC_Main", "RobotController Test Application Finished.");
    return 0;
}