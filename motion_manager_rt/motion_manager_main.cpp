// motion_manager_main.cpp
#include "MotionManager.h"
#include "InternalSimulation.h" // Используем симуляцию для этого теста
#include "MasterHardwareInterface.h"
#include "Logger.h"
#include "DataTypes.h"
#include "Units.h"
#include "KinematicModel.h" // Ensure this header defines KinematicModel

#include <iostream>
#include <thread>
#include <chrono>
#include <memory>
#include <iomanip>
#include <vector>
#include <string>

// Helper to print feedback with fault info
void printFeedbackPoint(const RDT::TrajectoryPoint& pt) {
    using namespace RDT;
    std::string fault_info = "";
    if (pt.feedback.fault.type == FaultType::VelocityLimit) {
        fault_info = " [WARNING: VELOCITY LIMIT ON A" + std::to_string(static_cast<int>(pt.feedback.fault.axis) + 1) +
                     ", req: " + pt.feedback.fault.actual_velocity_deg_s.toString(1) +
                     ", limit: " + pt.feedback.fault.limit_velocity_deg_s.toString(1) + "]";
    } else if (pt.feedback.fault.type == FaultType::PositionLimit) {
        fault_info = " [ERROR: POSITION LIMIT ON A" + std::to_string(static_cast<int>(pt.feedback.fault.axis) + 1) + "]";
    }

    LOG_INFO_F("FeedbackPrinter",
              "[FB] RTState: %d | A1 Cmd: %7.2f deg | A1 Fb: %7.2f deg | A2 Cmd: %7.2f deg | A2 Fb: %7.2f deg%s",
              static_cast<int>(pt.feedback.rt_state),
              pt.command.joint_target[AxisId::A1].angle.toDegrees().value(),
              pt.feedback.joint_actual[AxisId::A1].angle.toDegrees().value(),
              pt.command.joint_target[AxisId::A2].angle.toDegrees().value(),
              pt.feedback.joint_actual[AxisId::A2].angle.toDegrees().value(),
              fault_info.c_str()
    );
}

int main() {
    using namespace RDT;
    using namespace RDT::literals;
    using namespace std::chrono_literals;

    Logger::setLogLevel(LogLevel::Debug); 

    LOG_INFO("MM_StressTest", "--- The Ultimate MotionManager Stress Test ---");

    // --- 1. SETUP ---
    // Создаем конфиг, где реальный интерфейс не используется
    InterfaceConfig config;
    config.realtime_type = InterfaceConfig::RealtimeInterfaceType::None;
    config.simulation_initial_joints.fromAngleArray({0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad, 0.0_rad});

    // Создаем модель с тестовыми лимитами
    KinematicModel model = KinematicModel::createKR6R900(); // Берем геометрию, но лимиты переопределим
    RobotLimits limits;
    limits.joint_velocity_limits_deg_s.fill(180.0_deg_s);
    limits.joint_velocity_limits_deg_s[0] = 50.0_deg_s; // A1 медленная, для теста
    limits.joint_position_limits_deg.fill({-180.0_deg, 180.0_deg});
    limits.joint_position_limits_deg[0] = {-90.0_deg, 90.0_deg}; // A1 имеет жесткие лимиты

    // Собираем систему
    auto master_iface = std::make_shared<MasterHardwareInterface>(config);
    unsigned int cycle_period_ms = 20; // 50 Hz
    MotionManager mm(master_iface, cycle_period_ms, limits);

    if (!master_iface->connect() || !mm.start()) {
        LOG_CRITICAL("MM_StressTest", "System failed to start. Aborting.");
        return 1;
    }
    
    std::jthread feedback_consumer_thread([&mm](std::stop_token stoken) {
        while (!stoken.stop_requested()) {
            TrajectoryPoint fb_point;
            if (mm.dequeueFeedback(fb_point)) {
                printFeedbackPoint(fb_point);
            } else {
                std::this_thread::sleep_for(10ms); 
            }
        }
    });
    
    std::this_thread::sleep_for(200ms);

    // --- 2. VELOCITY LIMIT & SUBDIVISION TEST ---
    LOG_INFO("MM_StressTest", "\n--- TEST 1: VELOCITY LIMIT & SUBDIVISION ---");
    LOG_WARN("MM_StressTest", "A1 velocity limit is 50 deg/s. Sending command for 10 deg move in 20ms (=500 deg/s). Expecting subdivision into 10 steps.");
    TrajectoryPoint fast_pt;
    fast_pt.command.joint_target[AxisId::A1].angle = (10.0_deg).toRadians();
    if (!mm.enqueueCommand(fast_pt)) { LOG_ERROR("MM_StressTest", "Test 1 FAILED: Could not enqueue."); }
    LOG_INFO("MM_StressTest", "Waiting for subdivided move to complete...");
    std::this_thread::sleep_for(1s); // Даем время на выполнение 10 шагов по 20мс + запас

    // --- 3. POSITION LIMIT TEST ---
    LOG_INFO("MM_StressTest", "\n--- TEST 2: POSITION LIMIT ---");
    LOG_WARN("MM_StressTest", "A1 position limit is 90 deg. Sending command for 100 deg. Expecting ERROR state.");
    TrajectoryPoint illegal_pt;
    illegal_pt.command.joint_target[AxisId::A1].angle = (100.0_deg).toRadians();
    if (!mm.enqueueCommand(illegal_pt)) { LOG_ERROR("MM_StressTest", "Test 2 FAILED: Could not enqueue."); }
    std::this_thread::sleep_for(200ms);
    if (mm.getCurrentMode() != RTState::Error) {
        LOG_ERROR("MM_StressTest", "Test 2 FAILED: MotionManager did not enter Error state.");
    } else {
        LOG_INFO("MM_StressTest", "Test 2 PASSED: MotionManager correctly entered Error state.");
    }

    // --- 4. RESET TEST ---
    LOG_INFO("MM_StressTest", "\n--- TEST 3: RESET after error ---");
    mm.reset();
    std::this_thread::sleep_for(200ms);
     if (mm.getCurrentMode() != RTState::Idle) {
         LOG_ERROR("MM_StressTest", "Test 3 FAILED: MotionManager is not in Idle state after reset.");
    } else {
        LOG_INFO("MM_StressTest", "Test 3 PASSED: MotionManager is in Idle state.");
    }
    // Отправляем безопасную команду, чтобы убедиться, что система снова работает
    TrajectoryPoint recovery_pt;
    recovery_pt.command.joint_target[AxisId::A1].angle = (0.0_deg).toRadians(); // Возвращаемся в ноль
    if (!mm.enqueueCommand(recovery_pt)) { LOG_ERROR("MM_StressTest", "Test 3 FAILED: Could not enqueue after reset."); }
    std::this_thread::sleep_for(1s);

    // --- 5. E-STOP TEST ---
    LOG_INFO("MM_StressTest", "\n--- TEST 4: E-STOP during a long move ---");
    LOG_INFO("MM_StressTest", "Sending a long move (A2 -> 90 deg) that will be subdivided...");
    TrajectoryPoint long_move_pt;
    long_move_pt.command.joint_target[AxisId::A2].angle = (90.0_deg).toRadians();
    if (!mm.enqueueCommand(long_move_pt)) { LOG_ERROR("MM_StressTest", "Test 4 FAILED: Could not enqueue long move."); }
    
    std::this_thread::sleep_for(200ms); // Даем движению начаться и заполнить микро-очередь
    LOG_CRITICAL("MM_StressTest", ">>> TRIGGERING E-STOP NOW <<<");
    mm.emergencyStop();
    std::this_thread::sleep_for(200ms);
    
    if (mm.getCurrentMode() != RTState::Error) {
        LOG_ERROR("MM_StressTest", "E-Stop test FAILED: MotionManager is not in Error state.");
    } else {
        LOG_INFO("MM_StressTest", "E-Stop test PASSED: MotionManager is in Error state.");
    }
    // Проверяем, что обе очереди пусты (внутренняя недоступна, но внешняя должна быть пуста)
    if (mm.getCommandQueueSize() != 0) {
        LOG_ERROR("MM_StressTest", "E-Stop test FAILED: Main queue was not cleared.");
    } else {
        LOG_INFO("MM_StressTest", "E-Stop test PASSED: Main queue cleared.");
    }
    // Внутреннюю очередь мы проверим косвенно: после сброса робот не должен продолжать движение.
    
    // --- 6. QUEUE OVERFLOW TEST ---
    LOG_INFO("MM_StressTest", "\n--- TEST 5: QUEUE OVERFLOW ---");
    mm.reset(); // Сбрасываемся после E-Stop
    std::this_thread::sleep_for(200ms);
    LOG_WARN("MM_StressTest", "Spamming the main command queue with 300 commands...");
    int success_count = 0;
    for (int i = 0; i < 300; ++i) {
        TrajectoryPoint spam_pt;
        spam_pt.command.joint_target[AxisId::A3].angle = Degrees(i * 0.1).toRadians();
        if (mm.enqueueCommand(spam_pt)) {
            success_count++;
        }
    }
    LOG_INFO_F("MM_StressTest", "Successfully enqueued %d commands. Queue size: %zu.", success_count, mm.getCommandQueueSize());
    if (success_count > 0 && success_count < 300) {
        LOG_INFO("MM_StressTest", "Test 5 PASSED: Queue correctly reported overflow.");
    } else {
        LOG_ERROR("MM_StressTest", "Test 5 FAILED: Queue overflow behavior is unexpected.");
    }
    
    LOG_INFO("MM_StressTest", "Waiting for queue to drain...");
    // Ждем, пока MotionManager обработает все, что влезло в очередь
    while(mm.getCommandQueueSize() > 0 || mm.getCurrentMode() == RTState::Moving) {
        std::this_thread::sleep_for(100ms);
    }

    // --- Завершение ---
    LOG_INFO("MM_StressTest", "\n--- All tests complete. Shutting down. ---");
    feedback_consumer_thread.request_stop(); 
    mm.stop(); 

    if (feedback_consumer_thread.joinable()) {
        feedback_consumer_thread.join();
    }
    LOG_INFO("MM_StressTest", "All threads joined. Application finished.");

    return 0;
}