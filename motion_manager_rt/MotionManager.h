// MotionManager.h
#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#pragma once

#include "DataTypes.h"
#include "Units.h"
#include "TrajectoryQueue.h"
#include "Logger.h"
#include "KinematicModel.h"
#include "MasterHardwareInterface.h" // *** ЗАВИСИМОСТЬ ОТ ПРАВИЛЬНОГО ИНТЕРФЕЙСА ***

#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <deque>
#include <array>

namespace RDT {

class MotionManager {
public:
    MotionManager(std::shared_ptr<MasterHardwareInterface> hw_interface,
                  unsigned int cycle_period_ms,
                  const RobotLimits& limits);

    ~MotionManager();

    MotionManager(const MotionManager&) = delete;
    MotionManager& operator=(const MotionManager&) = delete;
    MotionManager(MotionManager&&) = delete;
    MotionManager& operator=(MotionManager&&) = delete;

    bool start();
    void stop();
    void emergencyStop();
    void reset();

    [[nodiscard]] bool enqueueCommand(const TrajectoryPoint& cmd_point);
    [[nodiscard]] bool dequeueFeedback(TrajectoryPoint& out_frame);

    [[nodiscard]] size_t getCommandQueueSize() const;
    [[nodiscard]] size_t getFeedbackQueueSize() const;
    [[nodiscard]] RTState getCurrentMode() const;

private:
    void rt_cycle_tick();
    void sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const;
    
    void serviceMainCommandQueue();
    void processMicroQueue();
    void subdivideAndFillBuffer(const AxisSet& start_joints, const TrajectoryPoint& target);
    void processIdleState();
    
    std::shared_ptr<MasterHardwareInterface> hw_interface_; 
    
    const std::chrono::milliseconds cycle_period_;
    const RobotLimits limits_;
    std::jthread rt_thread_;

    std::atomic<bool> running_{false};
    std::atomic<RTState> current_mode_{RTState::Idle};
    
    TrajectoryQueue<TrajectoryPoint, 256> command_queue_;
    TrajectoryQueue<TrajectoryPoint, 256> feedback_queue_;

    std::deque<TrajectoryPoint> rt_command_buffer_; // Микро-очередь
    AxisSet last_actual_joints_;
    
    static constexpr size_t RT_BUFFER_REFILL_THRESHOLD = 5;

    const std::string MODULE_NAME = "MotionManager";
};

} // namespace RDT
#endif // MOTION_MANAGER_H