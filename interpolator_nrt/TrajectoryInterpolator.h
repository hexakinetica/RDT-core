// TrajectoryInterpolator.h
#ifndef TRAJECTORYINTERPOLATOR_H
#define TRAJECTORYINTERPOLATOR_H

#pragma once

#include "MotionProfile.h" // RDT::MotionProfile and its derivatives
#include "DataTypes.h"     // RDT::TrajectoryPoint, MotionType, etc.
#include "Units.h"         // RDT::Seconds, etc.
#include "Logger.h"        // RDT::Logger

#include <memory>    // For std::unique_ptr
#include <string>    // For std::string in logs
#include <stdexcept> // For std::runtime_error, std::logic_error
#include <vector>    // Though not directly used in this header, often related

namespace RDT {

//      
using namespace RDT::literals;

// Default kinematic limits (using strong types)
//       ,  
//    speed_ratio/acceleration_ratio  TrajectoryPoint.
constexpr RadiansPerSecond DEFAULT_JOINT_V_MAX = 1.5_rad_s;     // : ~86 deg/s
constexpr RadiansPerSecondSq DEFAULT_JOINT_A_MAX = 3.0_rad_s2;  // :
constexpr MetersPerSecond DEFAULT_CART_V_MAX = 0.25_m_s;      // 250 mm/s
constexpr MetersPerSecondSq DEFAULT_CART_A_MAX = 0.5_m_s2;     // 500 mm/s^2

/**
 * @class TrajectoryInterpolator
 * @brief Manages motion profiles for trajectory segments and provides interpolated points.
 *
 * This class takes a start and target TrajectoryPoint for a segment,
 * creates an appropriate MotionProfile (e.g., JOINT, LIN based on trapezoidal math),
 * and then allows querying for interpolated TrajectoryPoints at given time steps (dt_step)
 * along that profile. It's designed to be used in a non-real-time context for
 * pre-calculating points or in a soft real-time context if performance allows.
 */
class TrajectoryInterpolator {
public:
    /**
     * @brief Default constructor. Initializes the interpolator to an idle state.
     */
    TrajectoryInterpolator();

    // Destructor (default is fine as std::unique_ptr handles its resource)
    ~TrajectoryInterpolator() = default;

    // Interpolator is typically stateful and manages a unique profile,
    // so making it non-copyable and non-movable is a safe default.
    TrajectoryInterpolator(const TrajectoryInterpolator&) = delete;
    TrajectoryInterpolator& operator=(const TrajectoryInterpolator&) = delete;
    TrajectoryInterpolator(TrajectoryInterpolator&&) = delete;
    TrajectoryInterpolator& operator=(TrajectoryInterpolator&&) = delete;

    /**
     * @brief Loads a new motion segment for interpolation based on a start and target TrajectoryPoint.
     * The type of motion (JOINT, LIN) is determined by target_for_segment.header.motion_type.
     * Speed and acceleration limits are taken from defaults and scaled by ratios in
     * target_for_segment.command.speed_ratio and target_for_segment.command.acceleration_ratio.
     *
     * @param start_of_segment The TrajectoryPoint defining the current state (start of the segment).
     *                         Its .command.joint_target and .command.cartesian_target are used as initial poses.
     * @param target_for_segment The TrajectoryPoint defining the desired target state of the segment.
     *                           Its .header.motion_type, .command target poses, and speed/accel ratios are used.
     * @throws std::invalid_argument if motion type is unsupported, if required poses are missing,
     *         or if effective speed/acceleration limits are non-positive for a segment that requires movement.
     * @throws std::logic_error if internal profile creation fails unexpectedly.
     */
    void loadSegment(const TrajectoryPoint& start_of_segment,
                     const TrajectoryPoint& target_for_segment);

    /**
     * @brief Calculates and returns the next interpolated TrajectoryPoint along the current segment.
     * This method advances the internal time of the interpolator by dt_step.
     * If dt_step is zero or negative, it returns the current interpolated point without advancing time.
     * If the profile has finished, it will repeatedly return the cached final point of the segment.
     *
     * @param dt_step The time increment for interpolation.
     * @return The interpolated TrajectoryPoint.
     * @throws std::logic_error if no motion profile is currently loaded.
     */
    [[nodiscard]] TrajectoryPoint nextPoint(Seconds dt_step);

    /**
     * @brief Checks if the current motion profile segment has completed its execution.
     * @return true if the current time in profile is at or beyond its calculated duration,
     *         or if no profile is currently loaded (considered idle).
     */
    [[nodiscard]] bool isIdle() const;

    /**
     * @brief Gets the current accumulated time within the loaded motion profile.
     * @return Seconds The current time. Returns 0.0_s if no profile is loaded.
     */
    [[nodiscard]] Seconds getCurrentTimeInProfile() const;

    /**
     * @brief Gets the total calculated duration of the currently loaded motion profile.
     * @return Seconds The total duration. Returns 0.0_s if no profile is loaded.
     */
    [[nodiscard]] Seconds getCurrentProfileDuration() const;

private:
    /**
     * @internal
     * @brief Generates a TrajectoryPoint for a specific time 't' within the current profile.
     * This method populates the .command part of the TrajectoryPoint with interpolated pose data.
     * Header information (tool, base, motion_type, speed/accel ratios) is copied from
     * the `segment_target_metadata_provider_`.
     * It marks the point as interpolated unless it's the final point of the segment
     * (which is handled by returning `segment_final_point_cache_` in `nextPoint`).
     *
     * @param t The absolute time within the current profile for which to generate the point.
     * @return The interpolated TrajectoryPoint.
     * @throws std::logic_error if `current_profile_` is null.
     * @throws std::runtime_error if the loaded profile type is unsupported by this generation logic.
     */
    [[nodiscard]] TrajectoryPoint generateTrajectoryPointForTime(Seconds t) const;

    std::unique_ptr<MotionProfile> current_profile_;    ///< The active motion profile for the current segment.
    Seconds time_in_current_profile_;                   ///< Accumulated time within the current_profile_.
    bool profile_loaded_;                               ///< True if a valid profile is currently loaded and ready for interpolation.

    // Caches metadata from the target waypoint of the segment. This metadata (tool, base, user comments,
    // original motion type, speed/accel ratios) is applied to all interpolated points generated for this segment.
    TrajectoryPoint segment_target_metadata_provider_;

    // Caches the state of the very last point of the current segment. This is generated once
    // when a segment is loaded and is used by nextPoint() to ensure the final interpolated
    // point precisely matches the target, avoiding floating-point accumulation errors.
    // It also correctly sets flags like is_target_reached_for_this_point.
    TrajectoryPoint segment_final_point_cache_;

    const std::string MODULE_NAME = "InterpolatorNRT"; // NRT for Non-Real-Time context
};

} // namespace RDT
#endif // TRAJECTORYINTERPOLATOR_H