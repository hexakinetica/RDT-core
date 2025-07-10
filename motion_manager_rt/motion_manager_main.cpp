// motion_manager_main.cpp
#include "MotionManager.h"
#include "FakeMotionInterface.h" // For instantiating a concrete IMotionInterface
#include "Logger.h"
#include "DataTypes.h" // For TrajectoryPoint, etc.
#include "Units.h"     // For literals like _ms, _deg, _rad, _m

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <iomanip> // For std::fixed, std::setprecision
#include <optional> // For std::optional from dequeueFeedback
#include <vector>   // For std::vector if creating arrays of points

// Helper to print TrajectoryPoint feedback (simplified)
void printFeedbackPoint(const RDT::TrajectoryPoint& pt) {
    using namespace RDT;
    // std::cout << std::fixed << std::setprecision(3); // Moved to a central place or use logger formatting
    // MODIFICATION: Using LOG_DEBUG_F for more structured output via logger
    LOG_DEBUG_F("FeedbackPrinter",
              "[FB] Seq: %u, TId: %llu, MType: %d, RTState: %d, Reach: %c, Err: %c | A1 Cmd: %s, A1 Fb: %s | ",
              pt.header.sequence_index,
              pt.header.trajectory_id,
              static_cast<int>(pt.header.motion_type),
              static_cast<int>(pt.feedback.rt_state),
              (pt.header.is_target_reached_for_this_point ? 'Y' : 'N'),
              (pt.header.has_error_at_this_point ? 'Y' : 'N'),
              pt.command.joint_target[AxisId::A1].angle.toDegrees().toString(1).c_str(), // toString(1) for 1 decimal place
              pt.feedback.joint_actual[AxisId::A1].angle.toDegrees().toString(1).c_str(),
              pt.command.cartesian_target.x.toString(3).c_str(), // toString(3) for 3 decimal places
              pt.feedback.cartesian_actual.x.toString(3).c_str()
    );
}


int main() {
    using namespace RDT;
    using namespace RDT::literals;
    using namespace std::chrono_literals;

    Logger::setLogLevel(LogLevel::Debug); 

    LOG_INFO("MM_TestMain", "MotionManager Test Application Starting...");

    auto fake_motion_iface = std::make_unique<FakeMotionInterface>();
    
    unsigned int cycle_period_ms = 500; 
    MotionManager mm(std::move(fake_motion_iface), cycle_period_ms);

    LOG_INFO("MM_TestMain", "Starting MotionManager...");
    mm.start(); 

    std::this_thread::sleep_for(500ms); 
    if (mm.getCurrentMode() == RTState::Error) {
        LOG_CRITICAL("MM_TestMain", "MotionManager failed to start properly or connect interface. Exiting.");
        mm.stop();
        return 1;
    }

    // Thread to consume feedback AND print queue sizes
    std::jthread feedback_consumer_thread([&mm](std::stop_token stoken) {
        LOG_INFO("FeedbackConsumer", "Feedback consumer thread started.");
        int print_counter = 0;
        while (!stoken.stop_requested()) {
            if (auto fb_point_opt = mm.dequeueFeedback()) {
                printFeedbackPoint(*fb_point_opt);
            } else {
                std::this_thread::sleep_for(10ms); 
            }

            // MODIFICATION: Print queue sizes periodically
            if (++print_counter % 50 == 0) { // Print every 50 * ~10ms = ~500ms
                LOG_INFO_F("QueueStatus", "CmdQueue: %zu, FbQueue: %zu",
                           mm.getCommandQueueSize(), mm.getFeedbackQueueSize());
            }
            // END MODIFICATION
        }

        LOG_INFO("FeedbackConsumer", std::format("Feedback consumer thread stopping. Final Queue Sizes - Cmd: {}, Fb: {}", mm.getCommandQueueSize(), mm.getFeedbackQueueSize()));
    });

    LOG_INFO_F("MM_TestMain", "Initial Queue Sizes - Cmd: %zu, Fb: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());

    LOG_INFO("MM_TestMain", "Enqueuing some JOINT commands...");
    for (int i = 0; i < 5; ++i) {
        TrajectoryPoint pt;
        pt.header.trajectory_id = 1;
        pt.header.sequence_index = i + 1;
        pt.header.motion_type = MotionType::JOINT;
        pt.header.segment_duration = Seconds((i+1) * 0.1);

        std::array<Radians, ROBOT_AXES_COUNT> target_angles;
        for(size_t j=0; j < ROBOT_AXES_COUNT; ++j) {
            target_angles[j] = Degrees(10.0 * (i + 1) * (j + 1) ).toRadians().normalized();
        }
        pt.command.joint_target.fromAngleArray(target_angles);
        pt.command.speed_ratio = 0.5 + (i * 0.1); 

        // MODIFICATION: Log queue sizes before pushing
        LOG_DEBUG_F("MM_TestMain", "Before Enqueue Cmd %d - CmdQueue: %zu, FbQueue: %zu",
                    i + 1, mm.getCommandQueueSize(), mm.getFeedbackQueueSize());
        // END MODIFICATION
        if (!mm.enqueueCommand(pt)) {
             LOG_ERROR_F("MM_TestMain", "Failed to enqueue command %d", i + 1);
        }
        // MODIFICATION: Log queue sizes after pushing
        LOG_DEBUG_F("MM_TestMain", "After Enqueue Cmd %d - CmdQueue: %zu, FbQueue: %zu",
                    i + 1, mm.getCommandQueueSize(), mm.getFeedbackQueueSize());
        // END MODIFICATION
        std::this_thread::sleep_for(cycle_period_ms * 3ms ); // Enqueue a bit slower than MM processes
    }

    LOG_INFO_F("MM_TestMain", "After enqueuing loop - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());
    LOG_INFO("MM_TestMain", "Waiting for commands to process...");
    std::this_thread::sleep_for(2s); // Increased wait time
    LOG_INFO_F("MM_TestMain", "After waiting - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());


    LOG_INFO("MM_TestMain", "Triggering Emergency Stop...");
    mm.emergencyStop();
    std::this_thread::sleep_for(500ms); 
    LOG_INFO_F("MM_TestMain", "After E-Stop - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());


    LOG_INFO("MM_TestMain", "Resetting MotionManager...");
    mm.reset();
    std::this_thread::sleep_for(500ms); 
    LOG_INFO_F("MM_TestMain", "After Reset - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());


    LOG_INFO("MM_TestMain", "Enqueuing one more command after reset...");
    TrajectoryPoint pt_after_reset;
    pt_after_reset.header.trajectory_id = 2;
    pt_after_reset.header.sequence_index = 1;
    pt_after_reset.command.joint_target[AxisId::A1].angle = (-15.0_deg).toRadians();
    //      
    uint64_t traj_id_log = pt_after_reset.header.trajectory_id;
    uint32_t seq_idx_log = pt_after_reset.header.sequence_index;
    if (!mm.enqueueCommand(pt_after_reset)) { // pt_after_reset    
        LOG_ERROR_F("MM_TestMain", "Failed to enqueue command after reset (traj_id: %llu, seq: %u).",
                    traj_id_log, 
                    seq_idx_log);
    }
    LOG_INFO_F("MM_TestMain", "After final enqueue - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());


    std::this_thread::sleep_for(2s); // Let it process
    LOG_INFO_F("MM_TestMain", "Before stop - CmdQueue: %zu, FbQueue: %zu", mm.getCommandQueueSize(), mm.getFeedbackQueueSize());


    LOG_INFO("MM_TestMain", "Stopping MotionManager and Feedback Consumer...");
    feedback_consumer_thread.request_stop(); 
    mm.stop(); 

    if (feedback_consumer_thread.joinable()) {
        feedback_consumer_thread.join();
    }
    LOG_INFO("MM_TestMain", "All threads joined. Application finished.");

    return 0;
}
