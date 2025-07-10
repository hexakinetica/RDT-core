// Panel_StateDisplay.h
#ifndef PANEL_STATEDISPLAY_H
#define PANEL_STATEDISPLAY_H

#pragma once

#include <QWidget>
#include <QLabel>
#include <QGridLayout>
#include <QGroupBox>
#include <array> // For std::array holding QLabels

//  RDT ,    
#include "DataTypes.h" // RDT::CartPose, RDT::AxisSet, RDT::ToolFrame, RDT::BaseFrame
#include "Units.h"     // RDT::Meters, RDT::Radians, RDT::Degrees, RDT::literals
#include "Logger.h"    //  ,  
#include "Mock_Adapter.h" // Mock adapter to simulate signals
namespace RDT {

/**
 * @file Panel_StateDisplay.h
 * @brief Defines the RDT::Panel_StateDisplay Qt widget for displaying robot state.
 */

/**
 * @class Panel_StateDisplay
 * @brief A Qt widget that displays various aspects of the robot's state.
 *
 * This panel shows the currently active tool and base frames, the TCP pose
 * (in millimeters and degrees) relative to the active base frame, and the
 * current joint angles (in degrees). It receives updates via Qt slots
 * taking RDT data types.
 */
class Panel_StateDisplay : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Constructor for Panel_StateDisplay.
     * @param parent Optional parent QWidget.
     */
    explicit Panel_StateDisplay(QWidget* parent = nullptr);
    ~Panel_StateDisplay() override = default;

    // Panel should be non-copyable and non-movable as per QObject guidelines
    Panel_StateDisplay(const Panel_StateDisplay&) = delete;
    Panel_StateDisplay& operator=(const Panel_StateDisplay&) = delete;
    Panel_StateDisplay(Panel_StateDisplay&&) = delete;
    Panel_StateDisplay& operator=(Panel_StateDisplay&&) = delete;

public slots:
    /**
     * @brief Slot to update the displayed TCP pose and joint angles.
     * @param tcp_in_current_base The robot's Tool Center Point (TCP) pose, expressed in the currently active base coordinate system.
     * @param joints The robot's current joint angles.
     */
    void onCurrentPoseUpdate(const RDT::CartPose& tcp_in_current_base, const RDT::AxisSet& joints);

    /**
     * @brief Slot to update the displayed active tool frame information.
     * @param tool The currently active RDT::ToolFrame.
     */
    void onActiveToolUpdate(const RDT::ToolFrame& tool);

    /**
     * @brief Slot to update the displayed active base frame information.
     * @param base The currently active RDT::BaseFrame.
     */
    void onActiveBaseUpdate(const RDT::BaseFrame& base);

    /**
     * @brief Slot to update displayed tool and base frame names directly from QStrings.
     * This can be used if the adapter sends names separately or if full Frame objects are not needed initially.
     * @param toolName The name of the active tool.
     * @param baseName The name of the active base frame.
     */
    void setActiveFrameNames(const QString& toolName, const QString& baseName);


private:
    void setupUI();            ///< Initializes and lays out the UI elements.
    void updateTCPDisplay();   ///< Updates TCP coordinate QLabels from cached_tcp_display_.
    void updateJointDisplay(); ///< Updates joint angle QLabels from cached_joints_display_.

    // UI Elements
    QLabel* tool_name_value_label_ = nullptr;
    QLabel* base_name_value_label_ = nullptr;

    std::array<QLabel*, 6> tcp_coord_labels_{}; // For "X:", "Y:", etc.
    std::array<QLabel*, 6> tcp_coord_values_{}; // For the numeric values of TCP
    std::array<QLabel*, 6> tcp_units_labels_{}; // For "[mm]", "[°]"

    std::array<QLabel*, 6> joint_angle_labels_{}; // For "A1:", "A2:", etc.
    std::array<QLabel*, 6> joint_angle_values_{}; // For the numeric values of joints
    std::array<QLabel*, 6> joint_units_labels_{}; // For "[°]"

    // Cached RDT data for display (to avoid frequent conversions if data hasn't changed)
    // These represent the data that *should* be displayed, after any necessary transformations by adapter.
    RDT::CartPose cached_tcp_display_;    ///< TCP pose in the context of cached_base_display_
    RDT::AxisSet cached_joints_display_;
    RDT::ToolFrame cached_tool_display_;  ///< The tool whose TCP is represented by cached_tcp_display_
    RDT::BaseFrame cached_base_display_;  ///< The base frame in which cached_tcp_display_ is expressed

    static inline const std::string MODULE_NAME = "StateDisplayPanel"; // For logging
};

} // namespace RDT
#endif // PANEL_STATEDISPLAY_H