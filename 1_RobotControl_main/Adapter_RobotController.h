// Adapter_RobotController.h
#ifndef ADAPTER_ROBOTCONTROLLER_H
#define ADAPTER_ROBOTCONTROLLER_H

#pragma once
#include <QObject>
#include <QString>
#include <QTimer>
#include <QVector>
#include <QMessageBox>
#include <memory>

#include "StateData.h"
#include "RobotController.h" 
#include "Panel_StateDisplay.h"
#include "Panel_RobotView3D.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"
#include "DataTypes.h"
#include "Logger.h"



namespace RDT {

/**
 * @class Adapter_RobotController
 * @brief The bridge between the non-Qt backend (RobotController) and the Qt GUI.
 *
 * This QObject-derived class translates GUI signals into backend method calls
 * and polls the backend's state to emit Qt signals for updating the GUI.
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

    // --- Connection Methods for Panels ---
    void connectStateDisplayPanel(RDT::Panel_StateDisplay* panel);
    void connectRobotView3D(Panel_RobotView3D* panel);
    void connectStatusBarPanel(RDT::Panel_StatusBar* panel);
    void connectTeachPanel(RDT::Panel_Teach* panel);
    void connectJogPanel(RDT::Panel_JogControl* panel);

    /** @brief Initializes the backend controller and starts the GUI update timer. */
    Q_INVOKABLE bool initializeBackend(const RDT::TrajectoryPoint& initial_robot_state_user);
    
    void stopGuiUpdateTimer();
    void startGuiUpdateTimer(int interval_ms = 100);

signals:
    // --- Signals for GUI Panels ---
    void currentPoseUpdated(const RDT::CartPose& tcp_in_active_base, const RDT::AxisSet& joints);
    void currentJointsChanged(const RDT::AxisSet& joints);
    void robotModeChanged(RDT::RobotMode mode);
    void systemMessageChanged(const QString& message, bool isError);
    void globalSpeedRatioChanged(double ratio_0_to_1);
    void estopStatusChanged(bool isActive);
    void physicalConnectionStatusChanged(bool isConnected);
    void programModelChanged(const QVector<RDT::TrajectoryPoint>& program);
    void activeModeChanged(const QString& modeName, bool isRealConnected);
    void programExecutionStepChanged(int current_step_index);
    void motionTaskActiveStatusChanged(bool isActive);
    void controllerInternalStateChanged(RDT::ControllerState state);

public slots:
    // --- Slots for Commands from GUI ---
    // Teach Panel
    void onTeachPanelTeachCurrentPose(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio);
    void onTeachPanelDeletePoint(int index);
    void onTeachPanelTouchUpPoint(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio);
    void onTeachPanelRunProgram();
    void onTeachPanelStopProgram();
    void onTeachPanelProgramSpeedChanged(int percent);
    void onTeachPanelActiveToolSelected(const QString& toolName);
    void onTeachPanelActiveBaseSelected(const QString& baseName);

    // Jog Panel
    void onJogPanelIncrementJoint(int axis_idx, double delta_deg, double speed_ratio);
    void onJogPanelIncrementCartesian(int axis_idx, double delta_val, const QString& frame_qstr, double speed_ratio);
    void onJogPanelGoHome();
    void onJogPanelEmergencyStop();
    void onJogPanelSpeedChanged(int percent);

    // Status Bar
    void onSwitchToSimRequested();
    void onSwitchToRealRequested();
    void onResetErrorRequested();

private slots:
    /** @brief The main polling function, called by a QTimer. */
    void pollStateDataAndUpdateGUI();

private:
    // Helper methods
    [[nodiscard]] RDT::ToolFrame getToolFrameFromGUIName(const QString& name) const;
    [[nodiscard]] RDT::BaseFrame getBaseFrameFromGUIName(const QString& name) const;
    void apply_cartesian_increment(CartPose& pose, int axis_idx, double increment);

    std::shared_ptr<RDT::RobotController> rc_impl_;
    std::shared_ptr<RDT::StateData> state_data_;
    QTimer* update_timer_;

    // --- Cached values to detect changes ---
    RDT::TrajectoryPoint last_polled_fb_tp_;
    RDT::RobotMode last_polled_robot_mode_ = RDT::RobotMode::Initializing;
    RDT::ControllerState last_polled_controller_state_ = RDT::ControllerState::Initializing;
    QString last_polled_system_message_;
    bool last_polled_has_error_ = true;
    double last_polled_speed_ratio_ = -1.0;
    bool last_polled_estop_state_ = true;
    bool last_polled_real_connected_ = false;
    bool last_polled_motion_task_active_ = true;
    QString last_polled_active_mode_;

    QVector<RDT::TrajectoryPoint> current_program_model_data_;

    static inline const std::string ADAPTER_MODULE_NAME = "RC_Adapter";
};

} // namespace RDT
#endif