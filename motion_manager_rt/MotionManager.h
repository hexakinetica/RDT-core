// MotionManager.h
#ifndef MOTION_MANAGER_H
#define MOTION_MANAGER_H

#pragma once

#include "DataTypes.h"         // RDT types (RobotCommandFrame, TrajectoryPoint, etc.)
#include "Units.h"             // RDT units (Radians, Seconds, etc.)
#include "TrajectoryQueue.h"   // RDT::TrajectoryQueue
#include "IMotionInterfaces.h" // RDT::IMotionInterface
#include "Logger.h"            // RDT::Logger

#include <thread>   // For std::jthread
#include <atomic>   // For std::atomic
#include <chrono>   // For std::chrono::milliseconds
#include <memory>   // For std::unique_ptr
#include <optional> // For std::optional
#include <mutex>    // For std::mutex (if needed for non-atomic shared data)
#include <string>   // For std::string

namespace RDT {

/**
 * @class MotionManager
 * @brief Manages the real-time motion execution cycle.
 *
 * MotionManager retrieves TrajectoryPoint commands from an input queue,
 * sends them to an IMotionInterface (robot hardware or simulator),
 * reads feedback, updates the TrajectoryPoint with this feedback,
 * and places it onto an output queue. It operates in its own thread.
 * It assumes TrajectoryPlanner provides RT-ready setpoints.
 */
class MotionManager {
public:
    /**
     * @brief Constructs a MotionManager.
     * @param motion_interface A unique_ptr to an IMotionInterface implementation.
     *                         The MotionManager takes ownership.
     * @param cycle_period_ms The desired period of the RT cycle in milliseconds.
     * @throws std::invalid_argument if motion_interface is null or cycle_period_ms is not positive.
     */
    MotionManager(std::unique_ptr<IMotionInterface> motion_interface,
                  unsigned int cycle_period_ms);
    ~MotionManager();

    // MotionManager is not copyable or movable due to thread and unique_ptr members.
    MotionManager(const MotionManager&) = delete;
    MotionManager& operator=(const MotionManager&) = delete;
    MotionManager(MotionManager&&) = delete;
    MotionManager& operator=(MotionManager&&) = delete;

    /** @brief Starts the RT execution loop in a new thread. */
    bool start();

    /** @brief Stops the RT execution loop and joins the thread. */
    void stop();

    /** @brief Triggers an emergency stop condition. */
    void emergencyStop();

    /** @brief Resets the MotionManager and the underlying motion interface. */
    void reset();

    /**
     * @brief Enqueues a command TrajectoryPoint for execution.
     * To be called by the TrajectoryPlanner or other command source.
     * @param cmd_point The TrajectoryPoint containing the command.
     * @return true if successfully enqueued, false if the command queue is full.
     */
    [[nodiscard]] bool enqueueCommand(const TrajectoryPoint& cmd_point);




    /**
     * @brief Dequeues a TrajectoryPoint containing feedback.
     * To be called by a consumer of feedback (e.g., GUI, logger).
     * @return std::optional<TrajectoryPoint> The feedback point if available, else std::nullopt.
     */
    [[nodiscard]] std::optional<TrajectoryPoint> dequeueFeedback();

    /**
     * @brief Gets the current number of commands in the input queue.
     * @return std::size_t The approximate size of the command queue.
     */
    [[nodiscard]] std::size_t getCommandQueueSize() const;

    /**
     * @brief Gets the current number of feedback points in the output queue.
     * @return std::size_t The approximate size of the feedback queue.
     */
    [[nodiscard]] std::size_t getFeedbackQueueSize() const;

    /**
     * @brief Gets the current operational mode of the MotionManager.
     * @return RTState The current state.
     */
    [[nodiscard]] RTState getCurrentMode() const;

    [[nodiscard]] std::string getActiveInterfaceName() const;
private:
    /** @brief The main real-time execution loop function. */
    void rt_cycle_tick();

    /**
     * @brief Pauses the current thread until the next RT cycle tick is due.
     * @param cycle_start_time The time_point when the current cycle began.
     */
    void sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const;

    /**
     * @brief Handles the logic for processing one command and getting feedback.
     * Called from within rt_cycle_tick.
     * @param command_to_process The TrajectoryPoint command to process.
     */
    [[nodiscard]] std::optional<TrajectoryPoint> sendToHALandGetFeedback(const TrajectoryPoint& point_to_update);

    /**
     * @brief Handles the idle state when no command is available.
     * Sends a "hold position" or current state command to the interface.
     */
    void processIdleState();


    std::unique_ptr<IMotionInterface> motion_interface_; ///< The interface to the robot/simulator.
    const std::chrono::milliseconds cycle_period_;       ///< RT cycle period.
    std::jthread rt_thread_;                             ///< Thread for the RT loop.

    std::atomic<bool> running_ = false;            ///< Flag to control the RT loop execution.
    std::atomic<bool> estop_active_ = false;       ///< Flag indicating E-Stop is active.

    // State variables protected by state_mutex_ if accessed by multiple threads
    // or if updates need to be atomic with respect to each other.
    // current_mode_ is atomic, message_ and error_code_ need protection if read externally.
    std::atomic<RTState> current_mode_ = RTState::Idle;
    // For simplicity, message and error_code are updated by MM thread only,
    // and could be read via a method that uses a mutex or returns a snapshot.
    // If only used for feedback_queue_, direct update by MM thread is fine.
    std::string current_status_message_; 
    int current_error_code_ = 0;       
    mutable std::mutex status_mutex_; // Protects current_status_message_ and current_error_code_

    // SPSC Queues for commands and feedback
    TrajectoryQueue<TrajectoryPoint, 256> command_queue_;  ///< Input queue for commands.
    TrajectoryQueue<TrajectoryPoint, 256> feedback_queue_; ///< Output queue for feedback.

    // State within the RT loop
    TrajectoryPoint current_rt_command_point_; ///< Holds the command currently being processed or last processed.
                                              ///< Used for "hold position" if queue is empty.
    bool has_active_rt_command_ = false;      ///< True if current_rt_command_point_ is valid for holding.

    const std::string MODULE_NAME = "MotionManager";
};

} // namespace RDT
#endif // MOTION_MANAGER_H
