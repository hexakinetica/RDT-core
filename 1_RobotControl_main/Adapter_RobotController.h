// Adapter_RobotController.h
#ifndef ADAPTER_ROBOTCONTROLLER_H
#define ADAPTER_ROBOTCONTROLLER_H

#pragma once
#include <QObject>
#include <QString>
#include <QTimer>
#include <QVector>
#include <memory>

//  RDT 
#include "StateData.h"
#include "RobotController.h" // RDT::RobotController  RDT::ControllerState
#include "DataTypes.h"     // RDT::TrajectoryPoint, ToolFrame, BaseFrame, etc.
#include "Units.h"         // RDT::literals
#include "Logger.h"
#include "KinematicSolver.h" // RDT::KinematicSolver

//        GUI
// ,        RDT
#include "Panel_StateDisplay.h"
#include "Panel_RobotView3D.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"


namespace RDT {

/**
 * @file Adapter_RobotController.h
 * @brief Defines the RDT::Adapter_RobotController Qt class, bridging the C++ backend with the Qt GUI.
 */

class Adapter_RobotController : public QObject {
    Q_OBJECT

public:
    explicit Adapter_RobotController(
        std::shared_ptr<RDT::RobotController> robotControllerImpl,
        std::shared_ptr<RDT::StateData> stateData,
        QObject* parent = nullptr);

    ~Adapter_RobotController() override;

    Adapter_RobotController(const Adapter_RobotController&) = delete;
    Adapter_RobotController& operator=(const Adapter_RobotController&) = delete;
    Adapter_RobotController(Adapter_RobotController&&) = delete;
    Adapter_RobotController& operator=(Adapter_RobotController&&) = delete;

    void connectStateDisplayPanel(RDT::Panel_StateDisplay* panel);
    void connectRobotView3D(Panel_RobotView3D* panel);
    void connectStatusBarPanel(RDT::Panel_StatusBar* panel);
    void connectTeachPanel(RDT::Panel_Teach* panel);
    void connectJogPanel(RDT::Panel_JogControl* panel);

    Q_INVOKABLE bool initializeBackendAndStartGUIUpdates(const RDT::TrajectoryPoint& initial_robot_state_user, int gui_update_interval_ms = 100);
    void stopGuiUpdateTimer();
    void startGuiUpdateTimer(int interval_ms = 100);

signals:
    void currentPoseUpdated(const RDT::CartPose& tcp_in_active_base, const RDT::AxisSet& joints);
    void currentJointsChanged(const RDT::AxisSet& joints);
    void activeToolFrameChanged(const RDT::ToolFrame& toolFrame);
    void activeBaseFrameChanged(const RDT::BaseFrame& baseFrame);
    void robotModeChanged(RDT::RobotMode mode);
    void controllerInternalStateChanged(RDT::ControllerState controller_state);
    void systemMessageChanged(const QString& message, bool isError);
    void globalSpeedRatioChanged(double ratio_0_to_1);
    void estopStatusChanged(bool isActive);
    void physicalConnectionStatusChanged(bool isConnected);
    void motionTaskActiveStatusChanged(bool isActive);
    void programExecutionStepChanged(int current_step_index);
    void programModelChanged(const QVector<RDT::TrajectoryPoint>& program);

public slots:
    // TeachPanel Slots
    void onTeachPanelTeachCurrentPose(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio);
    void onTeachPanelDeletePoint(int index);
    void onTeachPanelTouchUpPoint(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio);
    void onTeachPanelRunProgram();
    void onTeachPanelPauseResumeProgram();
    void onTeachPanelStopProgram();
    void onTeachPanelProgramSpeedChanged(int percent_1_to_100);
    void onTeachPanelActiveToolSelected(const QString& toolName);
    void onTeachPanelActiveBaseSelected(const QString& baseName);

    // JogControlPanel Slots
    void onJogPanelIncrementJoint(int axis_idx_0_based, double delta_deg_val, double speed_ratio_0_1);
    void onJogPanelIncrementCartesian(int axis_idx_0_based, double delta_val_mm_or_rad, const QString& frame_name_str, double speed_ratio_0_1);
    void onJogPanelContinuousJointStart(int axis_idx_0_based, bool positive_direction, double speed_ratio_0_1);
    void onJogPanelContinuousCartesianStart(int axis_idx_0_based, bool positive_direction, const QString& frame_name_str, double speed_ratio_0_1);
    void onJogPanelContinuousJointStop(int axis_idx_0_based);
    void onJogPanelContinuousCartesianStop(int axis_idx_0_based, const QString& frame_name_str);
    void onJogPanelGoHome();
    void onJogPanelEmergencyStop();
    void onJogPanelSpeedChanged(int percent_1_to_100);

    // StatusBarPanel Slot
    void onSimRealModeChanged(bool isRealModeSelected);

    // Slot for clearing errors from GUI
    void onClearRobotErrorRequested();


private slots:
    void pollStateDataAndUpdateGUI();

private:
    [[nodiscard]] RDT::ToolFrame getToolFrameFromGUIName(const QString& name) const;
    [[nodiscard]] RDT::BaseFrame getBaseFrameFromGUIName(const QString& name) const;

    std::shared_ptr<RDT::RobotController> rc_impl_;
    std::shared_ptr<RDT::StateData> state_data_;
    QTimer* update_timer_;

    RDT::Panel_StateDisplay* ui_state_panel_ptr_ = nullptr;
    Panel_RobotView3D*  ui_view_3d_ptr_     = nullptr;
    RDT::Panel_StatusBar*    ui_status_bar_ptr_  = nullptr;
    RDT::Panel_Teach*        ui_teach_panel_ptr_ = nullptr;
    RDT::Panel_JogControl*   ui_jog_panel_ptr_   = nullptr;

    RDT::TrajectoryPoint last_polled_fb_tp_;
    RDT::RobotMode last_polled_robot_mode_ = RDT::RobotMode::Initializing;
    RDT::ControllerState last_polled_controller_state_ = RDT::ControllerState::Initializing;
    QString last_polled_system_message_;
    bool last_polled_has_error_ = true;
    double last_polled_speed_ratio_ = -1.0;
    bool last_polled_estop_state_ = true;
    bool last_polled_phys_connection_ = true;
    bool last_polled_motion_task_active_ = true;
    RDT::ToolFrame last_polled_tool_frame_;
    RDT::BaseFrame last_polled_base_frame_;

    QVector<RDT::TrajectoryPoint> current_program_model_data_; //    

    static inline const std::string ADAPTER_MODULE_NAME = "RC_Adapter";
};

} // namespace RDT
#endif // ADAPTER_ROBOTCONTROLLER_H