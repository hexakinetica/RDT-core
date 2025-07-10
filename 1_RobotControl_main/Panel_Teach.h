// Panel_Teach.h
#ifndef PANEL_TEACH_H
#define PANEL_TEACH_H

#pragma once

#include <QWidget>
#include <QListWidget>
#include <QPushButton>
#include <QSlider>
#include <QLabel>
#include <QComboBox>
#include <QVector> //     QVector<RDT::TrajectoryPoint>

//  RDT 
#include "DataTypes.h" // RDT::TrajectoryPoint, CartPose, AxisSet, ToolFrame, BaseFrame, MotionType
#include "Units.h"     // RDT::literals
#include "Logger.h"    // RDT::Logger

namespace RDT {

/**
 * @file Panel_Teach.h
 * @brief Defines the RDT::Panel_Teach Qt widget for creating and managing robot programs.
 */

/**
 * @class Panel_Teach
 * @brief A Qt widget that allows users to create a sequence of TrajectoryPoints (a program),
 *        edit them, and control program execution.
 *
 * It interfaces with an Adapter_RobotController to get current robot state for teaching
 * points and to send program execution commands.
 */
class Panel_Teach : public QObject { // MODIFIED: QObject for signals/slots, parent usually a QWidget
    Q_OBJECT

public:
    /**
     * @brief Constructor for Panel_Teach.
     * @param parent Optional parent QWidget.
     */
    explicit Panel_Teach(QWidget* parent_widget = nullptr); // Changed parent type
    ~Panel_Teach() override = default;

    Panel_Teach(const Panel_Teach&) = delete;
    Panel_Teach& operator=(const Panel_Teach&) = delete;
    Panel_Teach(Panel_Teach&&) = delete;
    Panel_Teach& operator=(Panel_Teach&&) = delete;
    
    [[nodiscard]] QWidget* getWidget(); // Returns the main widget of the panel

    // ---      (,  Adapter_RobotController) ---
    /**
     * @brief Sets the currently selected tool in the Tool ComboBox.
     * @param toolName The name of the tool to select.
     */
    void setCurrentToolInComboBox(const QString& toolName);
    /**
     * @brief Sets the currently selected base in the Base ComboBox.
     * @param baseName The name of the base frame to select.
     */
    void setCurrentBaseInComboBox(const QString& baseName);

    /**
     * @brief Adds a TrajectoryPoint (representing a command) to the program list display.
     * @param tp The RDT::TrajectoryPoint to add.
     */
    void addTrajectoryPointToDisplay(const RDT::TrajectoryPoint& tp, const QString& pointName = "");

    /** @brief Clears all commands from the program list display. */
    void clearProgramDisplay();

    /**
     * @brief Refreshes the entire program list display with a new program.
     * @param program A QVector of RDT::TrajectoryPoint representing the program.
     */
    void refreshProgramDisplay(const QVector<RDT::TrajectoryPoint>& program);

    /**
     * @brief Highlights a specific command (by index) in the program list.
     * @param index The 0-based index of the command to highlight.
     */
    void highlightProgramStep(int index);

    // ---      ---
    /** @return The RDT::MotionType selected for a new point. */
    [[nodiscard]] RDT::MotionType getSelectedMotionTypeForNewPoint() const;
    /** @return The program execution speed as a percentage (1-100). */
    [[nodiscard]] int getProgramSpeedPercent() const;
    /** @return The name of the currently selected tool frame from the ComboBox. */
    [[nodiscard]] QString getCurrentToolNameFromComboBox() const;
    /** @return The name of the currently selected base frame from the ComboBox. */
    [[nodiscard]] QString getCurrentBaseNameFromComboBox() const;
    /** @return The currently selected row index in the program list. -1 if no selection. */
    [[nodiscard]] int getSelectedProgramStepIndex() const;

   // [[nodiscard]] QVector<RDT::TrajectoryPoint> getCurrentProgram() const;

signals:
    // ,   Panel  Adapter_RobotController
    // Adapter       RDT::RobotController
    //    .

    /** @brief User requests to teach (record) the current robot pose as a new program point.
     *  Adapter should get current robot pose, form a TrajectoryPoint with GUI settings,
     *  and potentially add it to a program list or send it for execution.
     */
    void teachCurrentPoseRequested(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio);

    /**
     * @brief User requests to delete the program point at the given index.
     * @param index 0-based index of the point to delete.
     */
    void deleteProgramPointRequested(int index);

    /**
     * @brief User requests to "touch-up" (re-teach) the pose of an existing program point
     *        to the current robot pose.
     * @param index 0-based index of the point to touch-up.
     * @param newToolName The tool currently selected in GUI for this touch-up.
     * @param newBaseName The base currently selected in GUI for this touch-up.
     */
    void touchUpProgramPointRequested(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio);

    /** @brief User requests to run the entire program. */
    void runProgramExecutionRequested();
    /** @brief User requests to pause or resume program execution. */
    void pauseResumeProgramExecutionRequested();
    /** @brief User requests to stop program execution. */
    void stopProgramExecutionRequested();

    /**
     * @brief Program execution speed override changed by the user.
     * @param speed_percent Speed as percentage (1-100).
     */
    void programSpeedOverrideChanged(int speed_percent);

    /**
     * @brief User selected a new active tool from the ComboBox.
     * @param toolName Name of the selected tool.
     */
    void activeToolSelectionChanged(const QString& toolName);

    /**
     * @brief User selected a new active base frame from the ComboBox.
     * @param baseName Name of the selected base frame.
     */
    void activeBaseSelectionChanged(const QString& baseName);

private slots:
    //      UI
    void onBtnTeachClicked();
    void onBtnDeleteClicked();
    void onBtnTouchUpClicked();
    void onBtnRunClicked();
    void onBtnPauseResumeClicked();
    void onBtnStopClicked();
    void onProgramSpeedSliderChanged(int value);
    void onToolComboBoxChanged(const QString& toolName);
    void onBaseComboBoxChanged(const QString& baseName);

private:
    void setupUI(); ///< Initializes and lays out the UI elements.
    /**
     * @brief Formats an RDT::TrajectoryPoint into a QString for display in the QListWidget.
     * @param tp The TrajectoryPoint to format.
     * @param pointName Optional name for the point (e.g., "P1", "Waypoint 5").
     * @return QString The display string.
     */
    [[nodiscard]] QString formatTrajectoryPointForDisplay(const RDT::TrajectoryPoint& tp, const QString& pointName = "") const;

    QWidget* main_widget_; // The actual QWidget for this panel

    // UI Elements
    QComboBox* tool_combo_ = nullptr;
    QComboBox* base_combo_ = nullptr;
    QListWidget* program_list_widget_ = nullptr;
    QSlider* program_speed_slider_ = nullptr;
    QLabel* program_speed_label_ = nullptr;
    QComboBox* motion_type_combo_ = nullptr; // For new points

    QPushButton* btn_add_point_ = nullptr;      // "Teach" button
    QPushButton* btn_delete_point_ = nullptr;
    QPushButton* btn_touch_up_point_ = nullptr;
    QPushButton* btn_edit_attribs_ = nullptr; // For editing speed, type of existing point (not pose)

    QPushButton* btn_run_program_ = nullptr;
    QPushButton* btn_pause_program_ = nullptr;
    QPushButton* btn_stop_program_ = nullptr;

    //   ,     .
    //     (, RobotController),      ,
    //      ,     refreshProgramDisplay.
    //   ,      .
    QVector<RDT::TrajectoryPoint> current_program_data_;

    static inline const std::string MODULE_NAME = "TeachPanel";
};

} // namespace RDT
#endif // PANEL_TEACH_H