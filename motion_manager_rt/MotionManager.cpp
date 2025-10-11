// MotionManager.cpp
#include "MotionManager.h"
#include <stdexcept>
#include <chrono>
#include <cmath>

namespace RDT {

using namespace std::chrono_literals;

MotionManager::MotionManager(std::shared_ptr<MasterHardwareInterface> hw_interface,
                             unsigned int cycle_period_ms,
                             const RobotLimits& limits)
    : hw_interface_(hw_interface),
      cycle_period_(cycle_period_ms),
      limits_(limits) {
    if (!hw_interface_) {
        throw std::invalid_argument("MotionManager: MasterHardwareInterface cannot be null.");
    }
    if (cycle_period_ms == 0) {
        throw std::invalid_argument("MotionManager: Invalid cycle period.");
    }
    LOG_INFO_F(MODULE_NAME, "MotionManager created with cycle period: %u ms.", cycle_period_ms);
}

MotionManager::~MotionManager() {
    stop();
}

bool MotionManager::start() {
    if (running_.load()) {
        LOG_WARN(MODULE_NAME, "RT Loop already running.");
        return false;
    }
    if (!hw_interface_->isConnected()) {
        LOG_ERROR(MODULE_NAME, "Hardware interface is not connected. Cannot start RT loop.");
        current_mode_.store(RTState::Error);
        return false;
    }
    LOG_INFO(MODULE_NAME, "Starting RT Loop...");
    running_ = true;
    current_mode_.store(RTState::Idle);
    rt_thread_ = std::jthread(&MotionManager::rt_cycle_tick, this);
    return true;
}

void MotionManager::stop() {
    running_ = false;
    if (rt_thread_.joinable()) {
        rt_thread_.request_stop();
        rt_thread_.join();
        LOG_INFO(MODULE_NAME, "RT Loop stopped and thread joined.");
    }
}

void MotionManager::emergencyStop() {
    LOG_CRITICAL(MODULE_NAME, "Emergency Stop requested! Clearing all command queues.");
    current_mode_.store(RTState::Error);
    command_queue_.clear();
    rt_command_buffer_.clear();
}

void MotionManager::reset() {
    LOG_INFO(MODULE_NAME, "Reset requested.");
    command_queue_.clear();
    rt_command_buffer_.clear();
    current_mode_.store(RTState::Idle);
}

bool MotionManager::enqueueCommand(const TrajectoryPoint& cmd_point) {
    if (!command_queue_.try_push(cmd_point)) {
        LOG_WARN(MODULE_NAME, "Main command queue overflow!");
        return false;
    }
    return true;
}

bool MotionManager::dequeueFeedback(TrajectoryPoint& out_frame) {
    return feedback_queue_.try_pop(out_frame);
}

size_t MotionManager::getCommandQueueSize() const { return command_queue_.size(); }
size_t MotionManager::getFeedbackQueueSize() const { return feedback_queue_.size(); }
RTState MotionManager::getCurrentMode() const { return current_mode_.load(); }

// --- Private Methods ---

void MotionManager::rt_cycle_tick() {
    LOG_INFO(MODULE_NAME, "RT thread started.");
    
    last_actual_joints_ = hw_interface_->getLatestFeedback();
    
    TrajectoryPoint feedback_point_template;

    while (!rt_thread_.get_stop_token().stop_requested() && running_.load()) {
        auto cycle_start_time = std::chrono::steady_clock::now();

        // 1. Всегда получаем самую свежую обратную связь
        last_actual_joints_ = hw_interface_->getLatestFeedback();

        // 2. Выполняем логику, только если нет ошибки
        if (current_mode_.load() != RTState::Error) {
             serviceMainCommandQueue();
             processMicroQueue();
        } else {
            // В режиме ошибки просто удерживаем позицию
            processIdleState();
        }

        // 3. Формируем и отправляем пакет обратной связи наверх
        feedback_point_template.feedback.joint_actual = last_actual_joints_;
        feedback_point_template.feedback.rt_state = current_mode_.load();
        // FaultData будет добавлено в feedback_point_template внутри serviceMainCommandQueue
        if (!feedback_queue_.try_push(feedback_point_template)) {
            LOG_WARN(MODULE_NAME, "Feedback queue full! A feedback point was dropped.");
        }
        // Очищаем FaultData после отправки
        feedback_point_template.feedback.fault.type = FaultType::None;

        sleepUntilNextCycle(cycle_start_time);
    }
    LOG_INFO(MODULE_NAME, "RT thread finishing.");
}

void MotionManager::serviceMainCommandQueue() {
    if (rt_command_buffer_.size() > RT_BUFFER_REFILL_THRESHOLD) {
        return;
    }

    TrajectoryPoint new_target;
    if (command_queue_.try_pop(new_target)) {
        current_mode_.store(RTState::Moving);
        
        // --- Проверка лимитов по положению ---
        for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            const auto& limit_pair_deg = limits_.joint_position_limits_deg[i];
            Degrees target_angle_deg = new_target.command.joint_target.at(i).angle.toDegrees();
            
            if (target_angle_deg < limit_pair_deg.first || target_angle_deg > limit_pair_deg.second) {
                LOG_ERROR_F(MODULE_NAME, "Target for Axis %zu (%s) is outside position limits! Motion stopped.",
                            i, target_angle_deg.toString().c_str());
                current_mode_.store(RTState::Error);
                // ... (здесь можно заполнить FaultData для отправки наверх) ...
                rt_command_buffer_.clear();
                command_queue_.clear();
                return;
            }
        }
        
        // --- Проверка скорости и "дробление" ---
        subdivideAndFillBuffer(last_actual_joints_, new_target);
    }
}

void MotionManager::processMicroQueue() {
    if (rt_command_buffer_.empty()) {
        processIdleState();
        return;
    }

    TrajectoryPoint point_to_execute = rt_command_buffer_.front();
    rt_command_buffer_.pop_front();
    
    if (hw_interface_->sendCommand(point_to_execute.command.joint_target) != CommStatus::Success) {
        LOG_ERROR(MODULE_NAME, "Failed to send command to hardware. Entering Error state.");
        current_mode_.store(RTState::Error);
        rt_command_buffer_.clear(); // Очищаем оставшиеся шаги
    }
}

void MotionManager::subdivideAndFillBuffer(const AxisSet& start_joints, const TrajectoryPoint& target) {
    const Seconds dt(std::chrono::duration<double>(cycle_period_).count());
    double max_ratio = 1.0;
    
    // Начальная точка для дробления - это либо последняя реальная позиция, либо последняя точка в нашей микро-очереди
    const AxisSet& subdivision_start_joints = rt_command_buffer_.empty() ? start_joints : rt_command_buffer_.back().command.joint_target;

    for (size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
        Radians delta = target.command.joint_target.at(i).angle - subdivision_start_joints.at(i).angle;
        RadiansPerSecond required_vel = delta / dt; 
        RadiansPerSecond limit_vel_rad_s = limits_.joint_velocity_limits_deg_s[i].toRadiansPerSecond();

        if (required_vel.abs() > limit_vel_rad_s) {
            double ratio = required_vel.abs().value() / limit_vel_rad_s.value();
            if (ratio > max_ratio) {
                max_ratio = ratio;
            }
        }
    }

    int num_steps = static_cast<int>(std::ceil(max_ratio));

    if (num_steps <= 1) {
        rt_command_buffer_.push_back(target);
        return;
    }
    
    LOG_WARN_F(MODULE_NAME, "Velocity limit exceeded. Subdividing move into %d steps.", num_steps);

    TrajectoryPoint intermediate_point = target; // Копируем метаданные
    const AxisSet target_joints = target.command.joint_target;

    for (int i = 1; i <= num_steps; ++i) {
        double alpha = static_cast<double>(i) / num_steps;
        AxisSet interpolated_joints;
        for (size_t j = 0; j < ROBOT_AXES_COUNT; ++j) {
            interpolated_joints.at(j).angle = subdivision_start_joints.at(j).angle + (target_joints.at(j).angle - subdivision_start_joints.at(j).angle) * alpha;
        }
        intermediate_point.command.joint_target = interpolated_joints;
        rt_command_buffer_.push_back(intermediate_point);
    }
}

void MotionManager::processIdleState() {
    if (current_mode_.load() != RTState::Idle) {
        current_mode_.store(RTState::Idle);
    }
    // В режиме Idle отправляем команду удержания текущей реальной позиции
    hw_interface_->sendCommand(last_actual_joints_);
}

void MotionManager::sleepUntilNextCycle(const std::chrono::steady_clock::time_point& cycle_start_time) const {
    auto now = std::chrono::steady_clock::now();
    auto elapsed = now - cycle_start_time;
    if (elapsed < cycle_period_) {
        std::this_thread::sleep_until(cycle_start_time + cycle_period_);
    } else {
        LOG_WARN_F(MODULE_NAME, "RT cycle overrun! Elapsed: %lld us, Period: %lld us",
                   std::chrono::duration_cast<std::chrono::microseconds>(elapsed).count(),
                   std::chrono::duration_cast<std::chrono::microseconds>(cycle_period_).count());
    }
}

} // namespace RDT