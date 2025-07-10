// Panel_JogControl.h
#ifndef PANEL_JOGCONTROL_H
#define PANEL_JOGCONTROL_H

#pragma once

#include <QWidget>
#include <QComboBox>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <array> // For std::array of QWidgets

//  RDT ,      
// (  ,    AxisId  ,  JogPanel     )
#include "DataTypes.h" // RDT::AxisId, RDT::ROBOT_AXES_COUNT
#include "Logger.h"    // RDT::Logger

namespace RDT {

/**
 * @file Panel_JogControl.h
 * @brief Defines the RDT::Panel_JogControl Qt widget for manual robot jogging.
 */

/**
 * @class Panel_JogControl
 * @brief A Qt widget providing UI elements for manual robot jogging in joint or Cartesian space.
 *
 * Users can select jog mode (Joint/Cartesian), coordinate frame for Cartesian jogging,
 * speed, and step size (for incremental jogging). It emits signals corresponding to
 * user actions, which are intended to be connected to an adapter or controller.
 */
class Panel_JogControl : public QWidget {
    Q_OBJECT

public:
    /**
     * @enum JogMode
     * @brief Defines the type of jogging.
     */
    enum class JogMode {
        Joint,      ///< Jogging individual robot joints.
        Cartesian   ///< Jogging the robot's TCP in a Cartesian coordinate system.
    };
    Q_ENUM(JogMode) // Makes enum usable in Qt's property system and meta-object system

    /**
     * @brief Constructor for Panel_JogControl.
     * @param parent Optional parent QWidget.
     */
    explicit Panel_JogControl(QWidget* parent = nullptr);
    ~Panel_JogControl() override = default;

    Panel_JogControl(const Panel_JogControl&) = delete;
    Panel_JogControl& operator=(const Panel_JogControl&) = delete;
    Panel_JogControl(Panel_JogControl&&) = delete;
    Panel_JogControl& operator=(Panel_JogControl&&) = delete;

signals:
    // --- Signals for Incremental Jogging ---
    /**
     * @brief Emitted when an incremental joint jog is requested.
     * @param axis_index_0_based The 0-based index of the joint to jog (0 to ROBOT_AXES_COUNT-1).
     * @param delta_value_rad The incremental change in radians.
     * @param speed_ratio_0_1 The speed factor for the jog (0.0 to 1.0).
     */
    void jogIncrementJointRequested(int axis_index_0_based, double delta_value_rad, double speed_ratio_0_1);

    /**
     * @brief Emitted when an incremental Cartesian jog is requested.
     * @param axis_index_0_based The 0-based index of the Cartesian axis (0-2 for X,Y,Z; 3-5 for Rx,Ry,Rz).
     * @param delta_value_mm_or_rad The incremental change (millimeters for X,Y,Z; radians for Rx,Ry,Rz).
     * @param frame_name_str Name of the coordinate frame ("Base", "Tool", "User") in which to jog.
     * @param speed_ratio_0_1 The speed factor for the jog (0.0 to 1.0).
     */
    void jogIncrementCartesianRequested(int axis_index_0_based, double delta_value_mm_or_rad, const QString& frame_name_str, double speed_ratio_0_1);

    /** @brief Emitted when continuous joint jogging starts for a specific axis and direction. */
    void jogContinuousJointStartRequested(int axis_index_0_based, bool positive_direction, double speed_ratio_0_1); // : jogContinuousStartRequested

    /** @brief Emitted when continuous Cartesian jogging starts for a specific axis and direction. */
    void jogContinuousCartesianStartRequested(int axis_index_0_based, bool positive_direction, const QString& frame_name_str, double speed_ratio_0_1); // : jogCartesianContinuousStartRequested

    /** @brief Emitted when continuous joint jogging stops for a specific axis. */
    void jogContinuousJointStopRequested(int axis_index_0_based); // : jogContinuousStopRequested

    /** @brief Emitted when continuous Cartesian jogging stops for a specific axis in a frame. */
    void jogContinuousCartesianStopRequested(int axis_index_0_based, const QString& frame_name_str); // : jogCartesianContinuousStopRequested

    // --- General Commands ---
    /** @brief Emitted when a "Go Home" action is requested. */
    void goHomeRequested();
    /** @brief Emitted when an Emergency Stop action is requested from this panel. */
    void emergencyStopRequested();
    /**
     * @brief Emitted when the jog speed slider value changes.
     * @param speed_percent The selected speed as a percentage (1 to 100).
     */
    void jogSpeedChanged(int speed_percent);

private slots:
    // Internal slots for UI interactions
    void onInternalModeToggle();
    void onInternalStepModeToggle();
    void onInternalJogButtonPressed(int axis_index, bool is_positive_direction);
    void onInternalJogButtonReleased(int axis_index, bool is_positive_direction); // Added direction for specific stop
    void onInternalSpeedSliderChanged(int value);

private:
    void setupUI();            ///< Initializes and lays out the UI elements.
    void updateButtonAndAxisLabels(); ///< Updates labels based on current_mode_ (Joint/Cartesian).
    void updateStepSizeSuffix();      ///< Updates suffix of step_size_spin_ (° or mm).

    // UI Elements
    JogMode current_jog_mode_ = JogMode::Joint;
    bool is_incremental_mode_ = false; // true = incremental, false = continuous

    QPushButton* mode_toggle_button_ = nullptr;    // Toggles between Joint and Cartesian
    QComboBox* jog_frame_combo_ = nullptr;         // Selects Base, Tool, User frame for Cartesian jog
    QSlider* jog_speed_slider_ = nullptr;      // Sets jog speed percentage
    QLabel* speed_display_label_ = nullptr;    // Displays current speed percentage
    QDoubleSpinBox* step_size_spin_ = nullptr; // Input for incremental step size
    QPushButton* step_mode_toggle_button_ = nullptr;// Toggles between Incremental and Continuous

    std::array<QLabel*, ROBOT_AXES_COUNT> axis_display_labels_{}; // A1/X, A2/Y, ...
    std::array<QPushButton*, ROBOT_AXES_COUNT> positive_jog_buttons_{};
    std::array<QPushButton*, ROBOT_AXES_COUNT> negative_jog_buttons_{};
    // Store which button is currently pressed for continuous jog stop
    std::array<bool, ROBOT_AXES_COUNT * 2> continuous_button_pressed_state_{}; // 0-5 for neg, 6-11 for pos


    QPushButton* go_home_button_ = nullptr;
    QPushButton* e_stop_button_ = nullptr;

    static inline const std::string MODULE_NAME = "JogPanel";
};

} // namespace RDT
#endif // PANEL_JOGCONTROL_H