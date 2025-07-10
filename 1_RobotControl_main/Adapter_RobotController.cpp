// Adapter_RobotController.cpp
#include "Adapter_RobotController.h"

// Panel headers required for connect statements
#include "Panel_StateDisplay.h"
#include "Panel_RobotView3D.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"

#include <QApplication> // For QApplication::processEvents() - use sparingly
#include <QDebug>       // For qDebug

namespace RDT {

using namespace RDT::literals;
using namespace std::chrono_literals;

Adapter_RobotController::Adapter_RobotController(
    std::shared_ptr<RDT::RobotController> robotControllerImpl,
    std::shared_ptr<RDT::StateData> stateData,
    QObject* parent)
    : QObject(parent),
      rc_impl_(std::move(robotControllerImpl)),
      state_data_(std::move(stateData)),
      update_timer_(new QTimer(this)) {

    if (!rc_impl_) {
        LOG_CRITICAL(ADAPTER_MODULE_NAME, "RDT::RobotController implementation cannot be null.");
        throw std::runtime_error("Adapter_RobotController: RobotController impl is null.");
    }
    if (!state_data_) {
        LOG_CRITICAL(ADAPTER_MODULE_NAME, "RDT::StateData instance cannot be null.");
        throw std::runtime_error("Adapter_RobotController: StateData instance is null.");
    }

    connect(update_timer_, &QTimer::timeout, this, &Adapter_RobotController::pollStateDataAndUpdateGUI);
    LOG_INFO(ADAPTER_MODULE_NAME, "Adapter_RobotController created.");
}

Adapter_RobotController::~Adapter_RobotController() {
    stopGuiUpdateTimer(); // Ensure timer is stopped
    LOG_INFO(ADAPTER_MODULE_NAME, "Adapter_RobotController destroyed.");
}

bool Adapter_RobotController::initializeBackendAndStartGUIUpdates(const RDT::TrajectoryPoint& initial_robot_state_user, int gui_update_interval_ms) {
    LOG_INFO(ADAPTER_MODULE_NAME, "Requesting backend initialization from Adapter.");
    if (!rc_impl_) {
        LOG_ERROR(ADAPTER_MODULE_NAME, "Backend not initialized: RobotController implementation is null.");
        return false;
    }
    
    bool success = rc_impl_->initialize(initial_robot_state_user);
    
    if (success) {
        LOG_INFO(ADAPTER_MODULE_NAME, "Backend RDT::RobotController initialized successfully.");
        pollStateDataAndUpdateGUI(); 
        startGuiUpdateTimer(gui_update_interval_ms);
    } else {
        LOG_ERROR_F(ADAPTER_MODULE_NAME, "Backend RDT::RobotController initialization FAILED. Error: %s",
                    state_data_ ? state_data_->getSystemMessage().c_str() : "Unknown (StateData null or RC init failed)");
        if (state_data_) { // Emit current (error) state to GUI
            emit systemMessageChanged(QString::fromStdString(state_data_->getSystemMessage()), state_data_->hasActiveError());
            emit robotModeChanged(state_data_->getRobotMode());
            if(rc_impl_) emit controllerInternalStateChanged(rc_impl_->getInternalControllerState());
        }
    }
    return success;
}

void Adapter_RobotController::startGuiUpdateTimer(int interval_ms) {
    if (update_timer_->isActive()) {
        update_timer_->stop();
    }
    int effective_interval = (interval_ms > 0) ? interval_ms : 100;
    update_timer_->start(effective_interval);
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI update timer started/restarted with interval %d ms.", effective_interval);
}

void Adapter_RobotController::stopGuiUpdateTimer() {
    if (update_timer_ && update_timer_->isActive()) { // Check if timer exists
        update_timer_->stop();
        LOG_INFO(ADAPTER_MODULE_NAME, "GUI update timer stopped.");
    }
}

// --- Connection to UI Panels ---
void Adapter_RobotController::connectStateDisplayPanel(RDT::Panel_StateDisplay* panel) {
    if (!panel) { LOG_WARN(ADAPTER_MODULE_NAME, "Attempted to connect null Panel_StateDisplay."); return; }
    ui_state_panel_ptr_ = panel;
    connect(this, &Adapter_RobotController::currentPoseUpdated,       ui_state_panel_ptr_, &RDT::Panel_StateDisplay::onCurrentPoseUpdate);
    connect(this, &Adapter_RobotController::activeToolFrameChanged,   ui_state_panel_ptr_, &RDT::Panel_StateDisplay::onActiveToolUpdate);
    connect(this, &Adapter_RobotController::activeBaseFrameChanged,   ui_state_panel_ptr_, &RDT::Panel_StateDisplay::onActiveBaseUpdate);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to Panel_StateDisplay.");
}
void Adapter_RobotController::connectRobotView3D(Panel_RobotView3D* panel) {
    if (!panel) { LOG_WARN(ADAPTER_MODULE_NAME, "Attempted to connect null Panel_RobotView3D."); return; }
    ui_view_3d_ptr_ = panel;
    connect(this, &Adapter_RobotController::currentJointsChanged, ui_view_3d_ptr_, &Panel_RobotView3D::update_jointpos);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to Panel_RobotView3D.");
}
void Adapter_RobotController::connectStatusBarPanel(RDT::Panel_StatusBar* panel) {
    if (!panel) { LOG_WARN(ADAPTER_MODULE_NAME, "Attempted to connect null Panel_StatusBar."); return; }
    ui_status_bar_ptr_ = panel;
    connect(this, &Adapter_RobotController::robotModeChanged,                 ui_status_bar_ptr_, &RDT::Panel_StatusBar::onSetRobotModeRDT);
    connect(this, &Adapter_RobotController::systemMessageChanged,             ui_status_bar_ptr_, &RDT::Panel_StatusBar::onSetSystemMessageRDT);
    connect(this, &Adapter_RobotController::globalSpeedRatioChanged,          ui_status_bar_ptr_, &RDT::Panel_StatusBar::onSetSpeedRatioRDT);
    connect(this, &Adapter_RobotController::estopStatusChanged,               ui_status_bar_ptr_, &RDT::Panel_StatusBar::onSetEstopStatusRDT);
    connect(this, &Adapter_RobotController::physicalConnectionStatusChanged,  ui_status_bar_ptr_, &RDT::Panel_StatusBar::onSetConnectionStatusRDT);
    connect(ui_status_bar_ptr_, &RDT::Panel_StatusBar::simRealModeChangedByUser, this, &Adapter_RobotController::onSimRealModeChanged);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to/from Panel_StatusBar.");
}
void Adapter_RobotController::connectTeachPanel(RDT::Panel_Teach* panel) {
    if (!panel) { LOG_WARN(ADAPTER_MODULE_NAME, "Attempted to connect null Panel_Teach."); return; }
    ui_teach_panel_ptr_ = panel;
    connect(panel, &RDT::Panel_Teach::teachCurrentPoseRequested,          this, &Adapter_RobotController::onTeachPanelTeachCurrentPose);
    connect(panel, &RDT::Panel_Teach::deleteProgramPointRequested,      this, &Adapter_RobotController::onTeachPanelDeletePoint);
    connect(panel, &RDT::Panel_Teach::touchUpProgramPointRequested,     this, &Adapter_RobotController::onTeachPanelTouchUpPoint);
    connect(panel, &RDT::Panel_Teach::runProgramExecutionRequested,     this, &Adapter_RobotController::onTeachPanelRunProgram);
    connect(panel, &RDT::Panel_Teach::pauseResumeProgramExecutionRequested, this, &Adapter_RobotController::onTeachPanelPauseResumeProgram);
    connect(panel, &RDT::Panel_Teach::stopProgramExecutionRequested,    this, &Adapter_RobotController::onTeachPanelStopProgram);
    connect(panel, &RDT::Panel_Teach::programSpeedOverrideChanged,      this, &Adapter_RobotController::onTeachPanelProgramSpeedChanged);
    connect(panel, &RDT::Panel_Teach::activeToolSelectionChanged,       this, &Adapter_RobotController::onTeachPanelActiveToolSelected);
    connect(panel, &RDT::Panel_Teach::activeBaseSelectionChanged,       this, &Adapter_RobotController::onTeachPanelActiveBaseSelected);
    connect(this, &Adapter_RobotController::programExecutionStepChanged, ui_teach_panel_ptr_, &RDT::Panel_Teach::highlightProgramStep);
    connect(this, &Adapter_RobotController::programModelChanged,        ui_teach_panel_ptr_, &RDT::Panel_Teach::refreshProgramDisplay);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to/from Panel_Teach.");
    if (state_data_ && ui_teach_panel_ptr_) { 
        ui_teach_panel_ptr_->setCurrentToolInComboBox(QString::fromStdString(state_data_->getActiveToolFrame().name));
        ui_teach_panel_ptr_->setCurrentBaseInComboBox(QString::fromStdString(state_data_->getActiveBaseFrame().name));
    }
}
void Adapter_RobotController::connectJogPanel(RDT::Panel_JogControl* panel) {
    if (!panel) { LOG_WARN(ADAPTER_MODULE_NAME, "Attempted to connect null Panel_JogControl."); return; }
    ui_jog_panel_ptr_ = panel;
    connect(panel, &RDT::Panel_JogControl::jogIncrementJointRequested,            this, &Adapter_RobotController::onJogPanelIncrementJoint);
    connect(panel, &RDT::Panel_JogControl::jogIncrementCartesianRequested,       this, &Adapter_RobotController::onJogPanelIncrementCartesian);
    connect(panel, &RDT::Panel_JogControl::jogContinuousJointStartRequested,     this, &Adapter_RobotController::onJogPanelContinuousJointStart);
    connect(panel, &RDT::Panel_JogControl::jogContinuousCartesianStartRequested, this, &Adapter_RobotController::onJogPanelContinuousCartesianStart);
    connect(panel, &RDT::Panel_JogControl::jogContinuousJointStopRequested,      this, &Adapter_RobotController::onJogPanelContinuousJointStop);
    connect(panel, &RDT::Panel_JogControl::jogContinuousCartesianStopRequested,  this, &Adapter_RobotController::onJogPanelContinuousCartesianStop);
    connect(panel, &RDT::Panel_JogControl::goHomeRequested,                     this, &Adapter_RobotController::onJogPanelGoHome);
    connect(panel, &RDT::Panel_JogControl::emergencyStopRequested,              this, &Adapter_RobotController::onJogPanelEmergencyStop);
    connect(panel, &RDT::Panel_JogControl::jogSpeedChanged,                     this, &Adapter_RobotController::onJogPanelSpeedChanged);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals from Panel_JogControl.");
}

// --- Polling Slot ---
void Adapter_RobotController::pollStateDataAndUpdateGUI() {
    if (!state_data_ || !rc_impl_) { return; }

    TrajectoryPoint sdata_fb_tp = state_data_->getFbTrajectoryPoint();
    RobotMode sdata_robot_m = state_data_->getRobotMode();
    ControllerState rc_internal_s = rc_impl_->getInternalControllerState();
    QString sdata_sys_msg = QString::fromStdString(state_data_->getSystemMessage());
    bool sdata_has_err = state_data_->hasActiveError();
    double sdata_g_speed_ratio = state_data_->getGlobalSpeedRatio();
    bool sdata_estop_s = state_data_->isEstopActive();
    bool sdata_phys_conn_s = state_data_->isPhysicallyConnected();
    bool rc_motion_active = rc_impl_->isMotionTaskActive();
    ToolFrame sdata_active_tool = state_data_->getActiveToolFrame();
    BaseFrame sdata_active_base = state_data_->getActiveBaseFrame();

    if (last_polled_fb_tp_.feedback.cartesian_actual != sdata_fb_tp.feedback.cartesian_actual ||
        last_polled_fb_tp_.feedback.joint_actual != sdata_fb_tp.feedback.joint_actual ||
        last_polled_tool_frame_ != sdata_active_tool || 
        last_polled_base_frame_ != sdata_active_base ) {
        emit currentPoseUpdated(sdata_fb_tp.feedback.cartesian_actual, sdata_fb_tp.feedback.joint_actual);
        if (last_polled_fb_tp_.feedback.joint_actual != sdata_fb_tp.feedback.joint_actual) {
            emit currentJointsChanged(sdata_fb_tp.feedback.joint_actual);
        }
        last_polled_fb_tp_.feedback.cartesian_actual = sdata_fb_tp.feedback.cartesian_actual;
        last_polled_fb_tp_.feedback.joint_actual = sdata_fb_tp.feedback.joint_actual;
    }
    if (last_polled_tool_frame_ != sdata_active_tool) {
        emit activeToolFrameChanged(sdata_active_tool);
        last_polled_tool_frame_ = sdata_active_tool;
    }
    if (last_polled_base_frame_ != sdata_active_base) {
        emit activeBaseFrameChanged(sdata_active_base);
        last_polled_base_frame_ = sdata_active_base;
    }
    if (last_polled_robot_mode_ != sdata_robot_m) {
        emit robotModeChanged(sdata_robot_m);
        last_polled_robot_mode_ = sdata_robot_m;
    }
    if (last_polled_controller_state_ != rc_internal_s) {
        emit controllerInternalStateChanged(rc_internal_s);
        last_polled_controller_state_ = rc_internal_s;
    }
    if (last_polled_system_message_ != sdata_sys_msg || last_polled_has_error_ != sdata_has_err) {
        emit systemMessageChanged(sdata_sys_msg, sdata_has_err);
        last_polled_system_message_ = sdata_sys_msg;
        last_polled_has_error_ = sdata_has_err;
    }
    if (std::abs(last_polled_speed_ratio_ - sdata_g_speed_ratio) > UnitConstants::DEFAULT_EPSILON) {
        emit globalSpeedRatioChanged(sdata_g_speed_ratio);
        last_polled_speed_ratio_ = sdata_g_speed_ratio;
    }
    if (last_polled_estop_state_ != sdata_estop_s) {
        emit estopStatusChanged(sdata_estop_s);
        last_polled_estop_state_ = sdata_estop_s;
    }
    if (last_polled_phys_connection_ != sdata_phys_conn_s) {
        emit physicalConnectionStatusChanged(sdata_phys_conn_s);
        last_polled_phys_connection_ = sdata_phys_conn_s;
    }
    if (last_polled_motion_task_active_ != rc_motion_active) {
        emit motionTaskActiveStatusChanged(rc_motion_active);
        last_polled_motion_task_active_ = rc_motion_active;
    }
}

// --- Slots Implementation for Commands from GUI ---
// (   ,   )

// TeachPanel Slots
void Adapter_RobotController::onTeachPanelTeachCurrentPose(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI TeachCurrentPose: Type %d, Tool '%s', Base '%s', Speed %.2f",
               static_cast<int>(type), toolName.toStdString().c_str(), baseName.toStdString().c_str(), speedRatio);
    if (!rc_impl_ || !state_data_ || !ui_teach_panel_ptr_) { /* log error, return */ return; }

    TrajectoryPoint current_fb_sdata = state_data_->getFbTrajectoryPoint();
    TrajectoryPoint taught_point_user_cmd;
    taught_point_user_cmd.header.motion_type = type;
    taught_point_user_cmd.header.tool = getToolFrameFromGUIName(toolName);
    taught_point_user_cmd.header.base = getBaseFrameFromGUIName(baseName);
    taught_point_user_cmd.command.speed_ratio = speedRatio;
    taught_point_user_cmd.header.data_type = (type == MotionType::JOINT || type == MotionType::PTP) ?
                                            WaypointDataType::JOINT_DOMINANT_CMD : WaypointDataType::CARTESIAN_DOMINANT_CMD;
    CartPose current_flange_in_robot_base;
    try {
        current_flange_in_robot_base = FrameTransformer::calculateFlangeInWorld(
            current_fb_sdata.feedback.cartesian_actual, current_fb_sdata.header.tool.transform);
    } catch (const std::exception& e) { /* log error, return */ return;}
    
    CartPose new_tool_tcp_in_robot_base = FrameTransformer::calculateTcpInWorld(
        current_flange_in_robot_base, taught_point_user_cmd.header.tool.transform);
    
    taught_point_user_cmd.command.cartesian_target = FrameTransformer::transformPoseFromWorld(
        new_tool_tcp_in_robot_base, taught_point_user_cmd.header.base.transform);
    taught_point_user_cmd.command.joint_target = current_fb_sdata.feedback.joint_actual;
    
    current_program_model_data_.append(taught_point_user_cmd);
    emit programModelChanged(current_program_model_data_);
    LOG_INFO_F(ADAPTER_MODULE_NAME, "Taught point generated. Program size: %d", current_program_model_data_.size());
}

void Adapter_RobotController::onTeachPanelDeletePoint(int index) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI DeletePoint: Idx %d", index);
    if (index >= 0 && index < current_program_model_data_.size()) {
        current_program_model_data_.remove(index);
        emit programModelChanged(current_program_model_data_);
    } else { LOG_WARN_F(ADAPTER_MODULE_NAME, "Delete invalid index: %d", index); }
}

void Adapter_RobotController::onTeachPanelTouchUpPoint(int index, RDT::MotionType type, const QString& tool, const QString& base, double speed) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI TouchUpPoint: Idx %d", index);
    if (index < 0 || index >= current_program_model_data_.size()) { /* log error, return */ return; }
    //    onTeachPanelTeachCurrentPose,   current_program_model_data_[index]
    // ... (  teach,     ) ...
    //  : emit programModelChanged(current_program_model_data_);
}

void Adapter_RobotController::onTeachPanelRunProgram() {
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI RunProgram");
    if (!rc_impl_) { LOG_ERROR(ADAPTER_MODULE_NAME, "RC not avail."); return; }
    if (current_program_model_data_.isEmpty()) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Program empty.");
        if(state_data_) state_data_->setSystemMessage("Program is empty.", false);
        return;
    }
    //     -   .
    //       RobotController  .
    LOG_INFO_F(ADAPTER_MODULE_NAME, "Executing first point of program (size: %d)", current_program_model_data_.size());
    rc_impl_->executeMotionToTarget(current_program_model_data_.first());
}
void Adapter_RobotController::onTeachPanelPauseResumeProgram() { LOG_WARN(ADAPTER_MODULE_NAME, "Pause/Resume Program: Not Implemented."); }
void Adapter_RobotController::onTeachPanelStopProgram() {
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI StopProgram");
    if(rc_impl_) rc_impl_->cancelCurrentMotion();
}
void Adapter_RobotController::onTeachPanelProgramSpeedChanged(int percent) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI ProgramSpeed: %d%%", percent);
    if (state_data_) state_data_->setGlobalSpeedRatio(static_cast<double>(percent) / 100.0);
}
void Adapter_RobotController::onTeachPanelActiveToolSelected(const QString& toolName) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI ToolSelected: %s", toolName.toStdString().c_str());
    if (state_data_) state_data_->setActiveToolFrame(getToolFrameFromGUIName(toolName));
}
void Adapter_RobotController::onTeachPanelActiveBaseSelected(const QString& baseName) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI BaseSelected: %s", baseName.toStdString().c_str());
    if (state_data_) state_data_->setActiveBaseFrame(getBaseFrameFromGUIName(baseName));
}

// JogControlPanel Slots
void Adapter_RobotController::onJogPanelIncrementJoint(int axis_idx, double delta_deg, double speed_ratio) {
    if (!rc_impl_ || !state_data_) return;
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI Jog Joint: Ax %d, Delta %.2f deg, Speed %.2f", axis_idx, delta_deg, speed_ratio);
    TrajectoryPoint fb_sdata = state_data_->getFbTrajectoryPoint();
    TrajectoryPoint jog_cmd;
    jog_cmd.header.motion_type = MotionType::PTP;
    jog_cmd.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    jog_cmd.header.tool = fb_sdata.header.tool; // Jog with current user tool
    jog_cmd.header.base = fb_sdata.header.base; // Jog with current user base
    jog_cmd.command.speed_ratio = std::max(0.01, speed_ratio);
    jog_cmd.command.acceleration_ratio = std::max(0.01, speed_ratio);
    AxisSet target_j = fb_sdata.feedback.joint_actual;
    if (axis_idx >= 0 && axis_idx < static_cast<int>(ROBOT_AXES_COUNT)) {
        target_j[static_cast<AxisId>(axis_idx)].angle += Degrees(delta_deg).toRadians();
    } else { LOG_ERROR_F(ADAPTER_MODULE_NAME, "Invalid axis idx %d for joint jog.", axis_idx); return; }
    jog_cmd.command.joint_target = target_j;
    rc_impl_->executeMotionToTarget(jog_cmd);
}
void Adapter_RobotController::onJogPanelIncrementCartesian(int axis_idx, double delta_val, const QString& frame_qstr, double speed_ratio) {
    if (!rc_impl_ || !state_data_) return;
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI Jog Cart: Ax %d, Delta %.2f, Frame '%s', Speed %.2f", axis_idx, delta_val, frame_qstr.toStdString().c_str(), speed_ratio);
    
    TrajectoryPoint fb_sdata = state_data_->getFbTrajectoryPoint(); // TCP in RobotBase, header has UserTool/UserBase
    TrajectoryPoint jog_cmd_user_frame; // Target will be TCP in SelectedJogFrame

    jog_cmd_user_frame.header.motion_type = MotionType::LIN;
    jog_cmd_user_frame.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    jog_cmd_user_frame.command.speed_ratio = std::max(0.01, speed_ratio);
    jog_cmd_user_frame.command.acceleration_ratio = std::max(0.01, speed_ratio);
    jog_cmd_user_frame.header.tool = fb_sdata.header.tool; // Jogging this TCP

    CartPose current_tcp_in_robot_base = fb_sdata.feedback.cartesian_actual;
    CartPose target_tcp_in_selected_jog_frame_coords; // This is the P_selectedJogFrame_newTCP

    BaseFrame selected_jog_frame_T_robotBase_selectedJogFrame; // T_robotBase_SelectedJogFrame
    if (frame_qstr.compare("Robot Base", Qt::CaseInsensitive) == 0) {
        selected_jog_frame_T_robotBase_selectedJogFrame = BaseFrame(CartPose{}, "JogInRobotBase");
        target_tcp_in_selected_jog_frame_coords = current_tcp_in_robot_base; // Current TCP is already in Robot Base
    } else if (frame_qstr.compare("Active Tool", Qt::CaseInsensitive) == 0) {
        selected_jog_frame_T_robotBase_selectedJogFrame.transform = current_tcp_in_robot_base; // Jog Frame IS the current TCP
        selected_jog_frame_T_robotBase_selectedJogFrame.name = "JogInActiveTool";
        target_tcp_in_selected_jog_frame_coords = CartPose{}; // Jogging from origin of Tool Frame
    } else if (frame_qstr.compare("Active User Base", Qt::CaseInsensitive) == 0) {
        selected_jog_frame_T_robotBase_selectedJogFrame = state_data_->getActiveBaseFrame(); // T_robotBase_ActiveUserBase
        target_tcp_in_selected_jog_frame_coords = FrameTransformer::transformPoseFromWorld(
            current_tcp_in_robot_base, selected_jog_frame_T_robotBase_selectedJogFrame.transform);
    } else { LOG_ERROR_F(ADAPTER_MODULE_NAME, "Unknown jog frame: %s", frame_qstr.toStdString().c_str()); return; }

    // Apply delta to the target_tcp_in_selected_jog_frame_coords
    if (axis_idx >= 0 && axis_idx < 3) { // X, Y, Z
        double current_val = target_tcp_in_selected_jog_frame_coords.get_value_at(axis_idx);
        target_tcp_in_selected_jog_frame_coords.set_value_at(axis_idx, current_val + Meters(delta_val / 1000.0).value());
    } else if (axis_idx >= 3 && axis_idx < 6) { // Rx, Ry, Rz
        double current_val = target_tcp_in_selected_jog_frame_coords.get_value_at(axis_idx);
        target_tcp_in_selected_jog_frame_coords.set_value_at(axis_idx, current_val + Degrees(delta_val).toRadians().value());
    } else { LOG_ERROR_F(ADAPTER_MODULE_NAME, "Invalid axis idx %d for Cart jog.", axis_idx); return; }

    jog_cmd_user_frame.command.cartesian_target = target_tcp_in_selected_jog_frame_coords;
    jog_cmd_user_frame.header.base = selected_jog_frame_T_robotBase_selectedJogFrame; // Target is defined in this frame

    const auto& pose1 = fb_sdata.command.cartesian_target;
    const auto& pose2 = jog_cmd_user_frame.command.cartesian_target;
    LOG_INFO_F(ADAPTER_MODULE_NAME, "fb_sdata.command.cartesian_target: [X=%.3f, Y=%.3f, Z=%.3f, Rx=%.3f, Ry=%.3f, Rz=%.3f]",
               pose1.x.value(), pose1.y.value(), pose1.z.value(),
               pose1.rx.value(), pose1.ry.value(), pose1.rz.value());
    LOG_INFO_F(ADAPTER_MODULE_NAME, "jog_cmd_user_frame.command.cartesian_target: [X=%.3f, Y=%.3f, Z=%.3f, Rx=%.3f, Ry=%.3f, Rz=%.3f]",
               pose2.x.value(), pose2.y.value(), pose2.z.value(),
               pose2.rx.value(), pose2.ry.value(), pose2.rz.value());
    rc_impl_->executeMotionToTarget(jog_cmd_user_frame);
}

void Adapter_RobotController::onJogPanelContinuousJointStart(int, bool, double) { LOG_WARN(ADAPTER_MODULE_NAME, "Cont. Joint Jog Start: NI"); }
void Adapter_RobotController::onJogPanelContinuousCartesianStart(int, bool, const QString&, double) { LOG_WARN(ADAPTER_MODULE_NAME, "Cont. Cart. Jog Start: NI"); }
void Adapter_RobotController::onJogPanelContinuousJointStop(int) { LOG_WARN(ADAPTER_MODULE_NAME, "Cont. Joint Jog Stop: NI"); }
void Adapter_RobotController::onJogPanelContinuousCartesianStop(int, const QString&) { LOG_WARN(ADAPTER_MODULE_NAME, "Cont. Cart. Jog Stop: NI"); }

void Adapter_RobotController::onJogPanelGoHome() {
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI GoHome");
    if (!rc_impl_ || !state_data_) {LOG_ERROR(ADAPTER_MODULE_NAME, "GoHome: RC/SD/Solver not avail."); return;}
    TrajectoryPoint tp_home;
    tp_home.header.motion_type = MotionType::PTP;
    tp_home.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    RDT::AxisSet home_pose_joint;
    std::array<RDT::Radians, RDT::ROBOT_AXES_COUNT> initial_angles = {RDT::Radians(0.0),RDT::Radians(0.0),RDT::Radians(0.0),RDT::Radians(0.0),RDT::Radians(0.0),RDT::Radians(0.0)};
    home_pose_joint.fromAngleArray(initial_angles);
    tp_home.command.joint_target = home_pose_joint;
    tp_home.command.speed_ratio = 0.25; // Safe speed
    tp_home.header.tool = state_data_->getActiveToolFrame(); // Use current tool context
    tp_home.header.base = state_data_->getActiveBaseFrame(); // Use current base context
    rc_impl_->executeMotionToTarget(tp_home);
}
void Adapter_RobotController::onJogPanelEmergencyStop() {
    LOG_CRITICAL(ADAPTER_MODULE_NAME, "GUI EStop (JogPanel)");
    if(rc_impl_) rc_impl_->emergencyStop();
}
void Adapter_RobotController::onJogPanelSpeedChanged(int percent) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI JogSpeed (Global): %d%%", percent);
    if (state_data_) state_data_->setGlobalSpeedRatio(static_cast<double>(percent) / 100.0);
}
void Adapter_RobotController::onSimRealModeChanged(bool isRealModeSelected) {
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI SimRealMode: %s", isRealModeSelected ? "REAL" : "SIM");
    if (!rc_impl_) return;
    // rc_impl_->setBackendType(isRealModeSelected ? DesiredBackend::Real : DesiredBackend::Sim);
    LOG_WARN(ADAPTER_MODULE_NAME, "Actual backend switching (SIM/REAL) in RobotController is TBD.");
}

void Adapter_RobotController::onClearRobotErrorRequested() {
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI Action: Clear Robot Error Requested.");
    if (rc_impl_) {
       // rc_impl_->clearActiveError(); // Call the public method in RobotController
        // Poll immediately to reflect changes if successful
        pollStateDataAndUpdateGUI();
    }
}


//   (,        )
ToolFrame Adapter_RobotController::getToolFrameFromGUIName(const QString& name_qstr) const {
    std::string name = name_qstr.toStdString();
    if (name == "Gripper") return ToolFrame(CartPose{0.0_m, 0.0_m, 0.20_m, (0.0_deg).toRadians(), (0.0_deg).toRadians(), (0.0_deg).toRadians()}, "Gripper");
    if (name == "Welder") return ToolFrame(CartPose{0.05_m, 0.0_m, 0.15_m, 0.0_rad, (10.0_deg).toRadians(), 0.0_rad}, "Welder");
    if (name == "ThinTool") return ToolFrame(CartPose{0.0_m, 0.0_m, 0.1_m}, "ThinTool");
    if (name == "DefaultTool@Init") return ToolFrame(CartPose{0.0_m, 0.0_m, 0.15_m}, "DefaultTool@Init");
    if (name == "SmallTool") return ToolFrame(CartPose{0.0_m, 0.0_m, 0.1_m}, "SmallTool");
    // Default case
    return ToolFrame(CartPose{}, "DefaultTool");
}
BaseFrame Adapter_RobotController::getBaseFrameFromGUIName(const QString& name_qstr) const {
    std::string name = name_qstr.toStdString();
    if (name == "UserBase1") return BaseFrame(CartPose{0.1_m, 0.1_m, 0.1_m}, "UserBase1");
    if (name == "FixtureA") return BaseFrame(CartPose{0.1_m, 0.1_m, 0.0_m}, "FixtureA");
    if (name == "TableSurface") return BaseFrame(CartPose{0.0_m, 0.0_m, -0.05_m}, "TableSurface");
    if (name == "RobotBase@Init") return BaseFrame(CartPose{}, "RobotBase@Init");
    // Default "RobotBase" is identity
    return BaseFrame(CartPose{}, "RobotBase");
}


} // namespace RDT