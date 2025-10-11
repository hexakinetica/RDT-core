// main.cpp (was test_kinematics.cpp)
// --- ВЕРСИЯ ДЛЯ ТЕСТА С БЕЗОПАСНЫМ API И ПОДРОБНЫМ ВЫВОДОМ ---

#include "KdlKinematicSolver.h"
#include "KinematicModel.h"
#include "DataTypes.h"
#include "Units.h"
#include "Logger.h"

// Используем пространства имен для удобства
using namespace RDT;
using namespace RDT::literals;

#include <iostream>
#include <memory>
#include <cmath>
#include <iomanip>
#include <vector>
#include <Eigen/Geometry> // Для сравнения ориентации

// ==========================================================================================
// ОСНОВНАЯ ФУНКЦИЯ ТЕСТА
// ==========================================================================================
int main([[maybe_unused]] int argc, [[maybe_unused]] char *argv[]) {
    RDT::Logger::setLogLevel(RDT::LogLevel::Info);

    LOG_INFO("KinematicsTest", "--- Step-by-Step LIN Motion Emulation Test (Safe API) ---");

    try {
        // 1. Инициализация
        LOG_INFO("KinematicsTest", "Creating KinematicModel and Solver...");
        KinematicModel model = KinematicModel::createKR6R900();
        KdlKinematicSolver solver(model);
        
        // ==================================================================================
        // ШАГ 0: Начальная позиция (все суставы в нулях)
        // ==================================================================================
        LOG_INFO("KinematicsTest", "\n--- STEP 0: Starting at Zero-Joint Position ---");
        
        AxisSet current_joints; // По умолчанию создается с нулями
        CartPose current_pose;
        if (!solver.solveFK(current_joints, current_pose)) {
            LOG_CRITICAL("KinematicsTest", "Initial FK failed. Aborting.");
            return 1;
        }

        LOG_INFO_F("KinematicsTest", "Initial Joints (deg): %s", current_joints.toJointPoseString().c_str());
        LOG_INFO_F("KinematicsTest", "Initial Cartesian Pose (FK): %s", current_pose.toDescriptiveString().c_str());

        // ==================================================================================
        // ШАГ 1: Движение по оси A1 на -2 градуса (суставное движение)
        // ==================================================================================
        LOG_INFO("KinematicsTest", "\n--- STEP 1: Moving A1 by -2 degrees (Joint Motion) ---");

        AxisSet target_joints_step1 = current_joints;
        target_joints_step1[AxisId::A1].angle = (-2.0_deg).toRadians();
        
        current_joints = target_joints_step1;
        if (!solver.solveFK(current_joints, current_pose)) {
            LOG_CRITICAL("KinematicsTest", "FK after joint move failed. Aborting.");
            return 1;
        }

        LOG_INFO_F("KinematicsTest", "New Joints (deg): %s", current_joints.toJointPoseString().c_str());
        LOG_INFO_F("KinematicsTest", "New Cartesian Pose (FK): %s", current_pose.toDescriptiveString().c_str());

        // ==================================================================================
        // ШАГ 2: Эмуляция линейного движения на +30мм по оси Y с заданным шагом
        // ==================================================================================
        LOG_INFO("KinematicsTest", "\n--- STEP 2: Emulating LIN motion by +30mm along Y-axis ---");

        const CartPose start_pose_step2 = current_pose;
        const AxisSet start_joints_step2 = current_joints;

        CartPose final_target_pose_step2 = start_pose_step2;
        final_target_pose_step2.y += (30.0_mm).toMeters();

        LOG_INFO_F("KinematicsTest", "Start Pose for LIN: %s", start_pose_step2.toDescriptiveString().c_str());
        LOG_INFO_F("KinematicsTest", "Final Target Pose:  %s", final_target_pose_step2.toDescriptiveString().c_str());

        const int num_steps = 30; 
        LOG_INFO_F("KinematicsTest", "Simulating trajectory with %d steps...", num_steps);
        
        AxisSet seed_joints_for_next_step = start_joints_step2;

        for (int i = 0; i <= num_steps; ++i) {
            double alpha = static_cast<double>(i) / num_steps;

            // 1. Вычисляем идеальную промежуточную декартову точку на прямой
            CartPose intermediate_target_pose;
            intermediate_target_pose.x = start_pose_step2.x;
            intermediate_target_pose.y = start_pose_step2.y + (final_target_pose_step2.y - start_pose_step2.y) * alpha;
            intermediate_target_pose.z = start_pose_step2.z;
            intermediate_target_pose.rx = start_pose_step2.rx;
            intermediate_target_pose.ry = start_pose_step2.ry;
            intermediate_target_pose.rz = start_pose_step2.rz;
            
            // 2. Решаем IK и проверяем статус
            IKResult ik_result = solver.solveIK(intermediate_target_pose, seed_joints_for_next_step);

            if (ik_result.status == IKStatus::Failed) {
                LOG_ERROR_F("KinematicsTest", "IK failed at step %d! Test aborted.", i);
                LOG_ERROR_F("KinematicsTest", "Failed to reach target pose: %s", intermediate_target_pose.toDescriptiveString().c_str());
                return 1;
            }
            
            // 3. Обновляем состояние для следующей итерации
            seed_joints_for_next_step = ik_result.solution;
            CartPose actual_pose_at_step;
            if (!solver.solveFK(seed_joints_for_next_step, actual_pose_at_step)) {
              //  LOG_ERROR_F("KinematicsTest", "Verification FK failed at step %d! Test aborted.", i);
                return 1;
            }

            // 4. Формируем строку статуса для вывода
            std::string status_str;
            switch (ik_result.status) {
                case IKStatus::Precise:     status_str = "Precise";     break;
                case IKStatus::Approximate: status_str = "APPROXIMATE"; break; // Выделяем для наглядности
                case IKStatus::Failed:      status_str = "FAILED";      break;
            }

            // 5. Выводим информацию в вашем формате
            std::cout << std::fixed << std::setprecision(4)
                      << "Step=" << std::setw(2) << i << " | "
                      << "Status=" << std::left << std::setw(12) << status_str << std::right << "| "
                      << "X(des|act)=" << std::setw(6) << intermediate_target_pose.x.toMillimeters().value() << "|" << std::setw(8) << actual_pose_at_step.x.toMillimeters().value() << " | "
                      << "Y=" << std::setw(6) << intermediate_target_pose.y.toMillimeters().value() << "|" << std::setw(8) << actual_pose_at_step.y.toMillimeters().value() << " | "
                      << "Z=" << std::setw(6) << intermediate_target_pose.z.toMillimeters().value() << "|" << std::setw(8) << actual_pose_at_step.z.toMillimeters().value() << " | "
                      << "A1=" << std::setw(6) << std::setprecision(3) << seed_joints_for_next_step[AxisId::A1].angle.toDegrees().value() << " | "
                      << "A2=" << std::setw(6) << seed_joints_for_next_step[AxisId::A2].angle.toDegrees().value() << " | "
                      << "A3=" << std::setw(6) << seed_joints_for_next_step[AxisId::A3].angle.toDegrees().value() << " | "
                      << "A4=" << std::setw(6) << seed_joints_for_next_step[AxisId::A4].angle.toDegrees().value() << " | "
                      << "A5=" << std::setw(6) << seed_joints_for_next_step[AxisId::A5].angle.toDegrees().value() << " | "
                      << "A6=" << std::setw(6) << seed_joints_for_next_step[AxisId::A6].angle.toDegrees().value()
                      << std::endl;
        }

        LOG_INFO("KinematicsTest", "Step-by-step LIN motion emulation finished.");
        
        // Финальная проверка
        CartPose final_actual_pose;
        if (!solver.solveFK(seed_joints_for_next_step, final_actual_pose)) {
            LOG_ERROR("KinematicsTest", "Final FK for error calculation failed!");
            return 1;
        }
        
        Meters dx = (final_actual_pose.x - final_target_pose_step2.x).abs();
        Meters dy = (final_actual_pose.y - final_target_pose_step2.y).abs();
        Meters dz = (final_actual_pose.z - final_target_pose_step2.z).abs();
        Meters final_error = Meters(std::sqrt(dx.value()*dx.value() + dy.value()*dy.value() + dz.value()*dz.value()));
        
        LOG_INFO_F("KinematicsTest", "Final Target Pose: %s", final_target_pose_step2.toDescriptiveString().c_str());
        LOG_INFO_F("KinematicsTest", "Final Actual Pose: %s", final_actual_pose.toDescriptiveString().c_str());
        LOG_INFO_F("KinematicsTest", "Total positional error: %s", final_error.toMillimeters().toString(6).c_str());

        // Увеличим допуск, так как мы разрешили приблизительные решения
        if (final_error > 0.001_m) { // Допуск 1 мм
            LOG_ERROR("KinematicsTest", "FAILURE: Final position error is too high!");
            return 1;
        } else {
            LOG_INFO("KinematicsTest", "SUCCESS: Final position is within acceptable tolerance for a robust solver.");
        }

    } catch (const std::exception& e) {
        LOG_CRITICAL_F("KinematicsTest", "A critical setup exception occurred: %s", e.what());
        return 1;
    }

    LOG_INFO("KinematicsTest", "\nAll sequential motion tests completed successfully.");
    return 0;
}