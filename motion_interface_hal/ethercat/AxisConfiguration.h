// AxisConfiguration.h
#ifndef AXIS_CONFIGURATION_H
#define AXIS_CONFIGURATION_H

#pragma once
#include "Units.h"

namespace RDT {

struct AxisConfiguration {
    // Параметры для преобразования единиц
    double gear_ratio = 1.0;
    int32_t encoder_resolution = 17;

    // Программные ограничения движения
    Degrees software_limit_min = Degrees(-180.0);
    Degrees software_limit_max = Degrees(180.0);

    // --- ЗНАЧЕНИЯ ДЛЯ ТЕСТА ---
    // Это значение будет использоваться для ограничения скорости
    DegreesPerSecond profile_velocity = DegreesPerSecond(0.0); // <-- ЦЕЛЕВОЕ ОГРАНИЧЕНИЕ 36 град/сек

    // Ускорения сделаем поменьше для плавности
    DegreesPerSecondSq profile_acceleration = DegreesPerSecondSq(180.0);
    DegreesPerSecondSq profile_deceleration = DegreesPerSecondSq(180.0);
    
    // Max Velocity пока не трогаем, так как не уверены в его работе
    DegreesPerSecond max_velocity = DegreesPerSecond(0.0); // Оставим большое значение

    // Ограничение по моменту
    uint16_t torque_limit = 100;

    // Ошибка слежения. 2 градуса.
    Degrees following_error_window = Degrees(20.0);
};

} // namespace RDT

#endif // AXIS_CONFIGURATION_H