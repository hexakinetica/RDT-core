// Adapter_RobotController.cpp
#include "Adapter_RobotController.h"
#include "Units.h"
#include "FrameTransformer.h"

// Include headers for panels to connect to them
#include "Panel_StateDisplay.h"
#include "Panel_RobotView3D.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"

namespace RDT {

using namespace RDT::literals;
using namespace std::chrono_literals;

Adapter_RobotController::Adapter_RobotController(
    std::shared_ptr<RDT::RobotController> robotControllerImpl,
    std::shared_ptr<RDT::StateData> stateData,
    QObject* parent)
    : QObject(parent),
      rc_impl_(robotControllerImpl),
      state_data_(std::move(stateData)),
      update_timer_(new QTimer(this)) {

    if (!rc_impl_) {
        LOG_CRITICAL(ADAPTER_MODULE_NAME, "RobotController implementation is null.");
    }
    if (!state_data_) {
        LOG_CRITICAL(ADAPTER_MODULE_NAME, "StateData instance is null.");
    }

    connect(update_timer_, &QTimer::timeout, this, &Adapter_RobotController::pollStateDataAndUpdateGUI);
    LOG_INFO(ADAPTER_MODULE_NAME, "Adapter_RobotController created.");
}

Adapter_RobotController::~Adapter_RobotController() {
    stopGuiUpdateTimer();
    LOG_INFO(ADAPTER_MODULE_NAME, "Adapter_RobotController destroyed.");
}

bool Adapter_RobotController::initializeBackend(const RDT::TrajectoryPoint& initial_robot_state_user) {
    LOG_INFO(ADAPTER_MODULE_NAME, "Requesting backend initialization from Adapter.");
    if (!rc_impl_) {
        LOG_ERROR(ADAPTER_MODULE_NAME, "Backend not initialized: RobotController is null.");
        return false;
    }
    
    bool success = rc_impl_->initialize(initial_robot_state_user);
    
    if (success) {
        LOG_INFO(ADAPTER_MODULE_NAME, "Backend initialized successfully.");
        pollStateDataAndUpdateGUI(); 
        startGuiUpdateTimer(100); // Start polling at 10 Hz
    } else {
        LOG_ERROR_F(ADAPTER_MODULE_NAME, "Backend initialization FAILED. Error: %s",
                    state_data_ ? state_data_->getSystemMessage().c_str() : "Unknown");
        if (state_data_) {
            emit systemMessageChanged(QString::fromStdString(state_data_->getSystemMessage()), state_data_->hasActiveError());
            emit robotModeChanged(state_data_->getRobotMode());
        }
    }
    return success;
}

void Adapter_RobotController::startGuiUpdateTimer(int interval_ms) {
    if (update_timer_->isActive()) {
        update_timer_->stop();
    }
    update_timer_->start(interval_ms > 0 ? interval_ms : 100);
    LOG_INFO_F(ADAPTER_MODULE_NAME, "GUI update timer started with interval %d ms.", update_timer_->interval());
}

void Adapter_RobotController::stopGuiUpdateTimer() {
    if (update_timer_ && update_timer_->isActive()) {
        update_timer_->stop();
        LOG_INFO(ADAPTER_MODULE_NAME, "GUI update timer stopped.");
    }
}

// --- Connection Methods ---
void Adapter_RobotController::connectStateDisplayPanel(RDT::Panel_StateDisplay* panel) {
    if (!panel) return;
    connect(this, &Adapter_RobotController::currentPoseUpdated, panel, &RDT::Panel_StateDisplay::onCurrentPoseUpdate);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to Panel_StateDisplay.");
}

void Adapter_RobotController::connectRobotView3D(Panel_RobotView3D* panel) {
    if (!panel) return;
    connect(this, &Adapter_RobotController::currentJointsChanged, panel, &Panel_RobotView3D::update_jointpos);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to Panel_RobotView3D.");
}

void Adapter_RobotController::connectStatusBarPanel(RDT::Panel_StatusBar* panel) {
    if (!panel) return;
    connect(this, &Adapter_RobotController::robotModeChanged,                panel, &RDT::Panel_StatusBar::onSetRobotModeRDT);
    connect(this, &Adapter_RobotController::systemMessageChanged,            panel, &RDT::Panel_StatusBar::onSetSystemMessageRDT);
    connect(this, &Adapter_RobotController::globalSpeedRatioChanged,         panel, &RDT::Panel_StatusBar::onSetSpeedRatioRDT);
    connect(this, &Adapter_RobotController::estopStatusChanged,              panel, &RDT::Panel_StatusBar::onSetEstopStatusRDT);
    connect(this, &Adapter_RobotController::physicalConnectionStatusChanged, panel, &RDT::Panel_StatusBar::onSetConnectionStatusRDT);
    connect(this, &Adapter_RobotController::activeModeChanged,               panel, &RDT::Panel_StatusBar::onActiveModeChanged);
    
    connect(panel, &RDT::Panel_StatusBar::switchToSimRequested,  this, &Adapter_RobotController::onSwitchToSimRequested);
    connect(panel, &RDT::Panel_StatusBar::switchToRealRequested, this, &Adapter_RobotController::onSwitchToRealRequested);
    connect(panel, &RDT::Panel_StatusBar::resetErrorRequested,   this, &Adapter_RobotController::onResetErrorRequested);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to/from Panel_StatusBar.");
}

void Adapter_RobotController::connectTeachPanel(RDT::Panel_Teach* panel) {
    if (!panel) return;
    connect(panel, &RDT::Panel_Teach::teachCurrentPoseRequested, this, &Adapter_RobotController::onTeachPanelTeachCurrentPose);
    connect(panel, &RDT::Panel_Teach::deleteProgramPointRequested, this, &Adapter_RobotController::onTeachPanelDeletePoint);
    connect(panel, &RDT::Panel_Teach::touchUpProgramPointRequested, this, &Adapter_RobotController::onTeachPanelTouchUpPoint);
    connect(panel, &RDT::Panel_Teach::runProgramExecutionRequested, this, &Adapter_RobotController::onTeachPanelRunProgram);
    connect(panel, &RDT::Panel_Teach::stopProgramExecutionRequested, this, &Adapter_RobotController::onTeachPanelStopProgram);
    connect(panel, &RDT::Panel_Teach::programSpeedOverrideChanged, this, &Adapter_RobotController::onTeachPanelProgramSpeedChanged);
    connect(panel, &RDT::Panel_Teach::activeToolSelectionChanged, this, &Adapter_RobotController::onTeachPanelActiveToolSelected);
    connect(panel, &RDT::Panel_Teach::activeBaseSelectionChanged, this, &Adapter_RobotController::onTeachPanelActiveBaseSelected);
    connect(this, &Adapter_RobotController::programModelChanged, panel, &RDT::Panel_Teach::refreshProgramDisplay);
    connect(this, &Adapter_RobotController::programExecutionStepChanged, panel, &RDT::Panel_Teach::highlightProgramStep);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals to/from Panel_Teach.");
}

void Adapter_RobotController::connectJogPanel(RDT::Panel_JogControl* panel) {
    if (!panel) return;
    connect(panel, &RDT::Panel_JogControl::jogIncrementJointRequested, this, &Adapter_RobotController::onJogPanelIncrementJoint);
    connect(panel, &RDT::Panel_JogControl::jogIncrementCartesianRequested, this, &Adapter_RobotController::onJogPanelIncrementCartesian);
    connect(panel, &RDT::Panel_JogControl::goHomeRequested, this, &Adapter_RobotController::onJogPanelGoHome);
    connect(panel, &RDT::Panel_JogControl::emergencyStopRequested, this, &Adapter_RobotController::onJogPanelEmergencyStop);
    connect(panel, &RDT::Panel_JogControl::jogSpeedChanged, this, &Adapter_RobotController::onJogPanelSpeedChanged);
    LOG_INFO(ADAPTER_MODULE_NAME, "Connected signals from Panel_JogControl.");
}


// --- Main GUI Polling Loop ---
void Adapter_RobotController::pollStateDataAndUpdateGUI() {
    if (!state_data_ || !rc_impl_) { return; }

    TrajectoryPoint sdata_fb_tp = state_data_->getFbTrajectoryPoint();
    if (last_polled_fb_tp_.feedback.joint_actual != sdata_fb_tp.feedback.joint_actual) {
        emit currentJointsChanged(sdata_fb_tp.feedback.joint_actual);
        emit currentPoseUpdated(sdata_fb_tp.feedback.cartesian_actual, sdata_fb_tp.feedback.joint_actual);
        last_polled_fb_tp_ = sdata_fb_tp;
    }

    RobotMode sdata_robot_m = state_data_->getRobotMode();
    if (last_polled_robot_mode_ != sdata_robot_m) {
        emit robotModeChanged(sdata_robot_m);
        last_polled_robot_mode_ = sdata_robot_m;
    }
    
    ControllerState rc_internal_s = rc_impl_->getInternalControllerState();
    if (last_polled_controller_state_ != rc_internal_s) {
        emit controllerInternalStateChanged(rc_internal_s);
        last_polled_controller_state_ = rc_internal_s;
    }

    QString sdata_sys_msg = QString::fromStdString(state_data_->getSystemMessage());
    bool sdata_has_err = state_data_->hasActiveError();
    if (last_polled_system_message_ != sdata_sys_msg || last_polled_has_error_ != sdata_has_err) {
        emit systemMessageChanged(sdata_sys_msg, sdata_has_err);
        last_polled_system_message_ = sdata_sys_msg;
        last_polled_has_error_ = sdata_has_err;
    }
    
    double sdata_g_speed_ratio = state_data_->getGlobalSpeedRatio();
    if (std::abs(last_polled_speed_ratio_ - sdata_g_speed_ratio) > UnitConstants::DEFAULT_EPSILON) {
        emit globalSpeedRatioChanged(sdata_g_speed_ratio);
        last_polled_speed_ratio_ = sdata_g_speed_ratio;
    }

    bool sdata_estop_s = state_data_->isEstopActive();
    if (last_polled_estop_state_ != sdata_estop_s) {
        emit estopStatusChanged(sdata_estop_s);
        last_polled_estop_state_ = sdata_estop_s;
    }

    bool rc_motion_active = rc_impl_->isMotionTaskActive();
    if (last_polled_motion_task_active_ != rc_motion_active) {
        emit motionTaskActiveStatusChanged(rc_motion_active);
        last_polled_motion_task_active_ = rc_motion_active;
    }

    bool is_real_hw_connected = rc_impl_->isRealInterfaceConnected();
    auto current_mode = rc_impl_->getActiveMode();
    QString mode_str = (current_mode == MasterHardwareInterface::ActiveMode::Simulation) ? "Simulation" : "Realtime";
    
    if (last_polled_active_mode_ != mode_str || last_polled_real_connected_ != is_real_hw_connected) {
        emit activeModeChanged(mode_str, is_real_hw_connected);
        emit physicalConnectionStatusChanged(is_real_hw_connected);
        last_polled_active_mode_ = mode_str;
        last_polled_real_connected_ = is_real_hw_connected;
    }
}

// --- Slots for Commands from GUI ---
void Adapter_RobotController::onTeachPanelTeachCurrentPose(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio) {
    if (!state_data_) return;
    TrajectoryPoint taught_point;
    taught_point.header.motion_type = type;
    taught_point.header.tool = getToolFrameFromGUIName(toolName);
    taught_point.header.base = getBaseFrameFromGUIName(baseName);
    taught_point.command.speed_ratio = speedRatio;

    TrajectoryPoint current_fb = state_data_->getFbTrajectoryPoint();
    taught_point.command.joint_target = current_fb.feedback.joint_actual;
    // The taught cartesian pose needs to be expressed relative to the selected base frame
    taught_point.command.cartesian_target = FrameTransformer::transformPoseFromWorld(
        current_fb.feedback.cartesian_actual,
        taught_point.header.base.transform
    );
    
    current_program_model_data_.append(taught_point);
    emit programModelChanged(current_program_model_data_);
}

void Adapter_RobotController::onTeachPanelDeletePoint(int index) {
    if (index >= 0 && index < current_program_model_data_.size()) {
        current_program_model_data_.remove(index);
        emit programModelChanged(current_program_model_data_);
    }
}

void Adapter_RobotController::onTeachPanelTouchUpPoint(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio) {
    if (index < 0 || index >= current_program_model_data_.size()) return;
    
    TrajectoryPoint& point_to_update = current_program_model_data_[index];
    point_to_update.header.motion_type = type;
    point_to_update.header.tool = getToolFrameFromGUIName(newToolName);
    point_to_update.header.base = getBaseFrameFromGUIName(newBaseName);
    point_to_update.command.speed_ratio = speedRatio;

    TrajectoryPoint current_fb = state_data_->getFbTrajectoryPoint();
    point_to_update.command.joint_target = current_fb.feedback.joint_actual;
    point_to_update.command.cartesian_target = FrameTransformer::transformPoseFromWorld(
        current_fb.feedback.cartesian_actual,
        point_to_update.header.base.transform
    );

    emit programModelChanged(current_program_model_data_);
}

void Adapter_RobotController::onTeachPanelRunProgram() {
    if (!rc_impl_ || current_program_model_data_.isEmpty()) return;
    // For now, just execute the first point as a single motion task
    if (!rc_impl_->executeMotionToTarget(current_program_model_data_.first())) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Could not start program execution.");
    }
}

void Adapter_RobotController::onTeachPanelStopProgram() {
    if(rc_impl_) rc_impl_->cancelCurrentMotion();
}

void Adapter_RobotController::onTeachPanelProgramSpeedChanged(int percent) {
    if (state_data_) state_data_->setGlobalSpeedRatio(static_cast<double>(percent) / 100.0);
}

void Adapter_RobotController::onTeachPanelActiveToolSelected(const QString& toolName) {
    if (state_data_) state_data_->setActiveToolFrame(getToolFrameFromGUIName(toolName));
}

void Adapter_RobotController::onTeachPanelActiveBaseSelected(const QString& baseName) {
    if (state_data_) state_data_->setActiveBaseFrame(getBaseFrameFromGUIName(baseName));
}

void Adapter_RobotController::onJogPanelIncrementJoint(int axis_idx, double delta_deg, double speed_ratio) {
    if (!rc_impl_ || !state_data_) return;
    
    TrajectoryPoint jog_cmd;
    jog_cmd.header.motion_type = MotionType::PTP;
    jog_cmd.header.data_type = WaypointDataType::JOINT_DOMINANT_CMD;
    jog_cmd.header.tool = state_data_->getActiveToolFrame();
    jog_cmd.header.base = state_data_->getActiveBaseFrame();
    jog_cmd.command.speed_ratio = std::max(0.01, speed_ratio);
    
    AxisSet target_j = state_data_->getFbTrajectoryPoint().feedback.joint_actual;
    if (axis_idx >= 0 && axis_idx < static_cast<int>(ROBOT_AXES_COUNT)) {
        target_j.at(axis_idx).angle += Degrees(delta_deg).toRadians();
    } else { return; }
    jog_cmd.command.joint_target = target_j;
    
    if (!rc_impl_->executeMotionToTarget(jog_cmd)) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Could not execute joint jog.");
    }
}

void Adapter_RobotController::onJogPanelIncrementCartesian(int axis_idx, double delta_val, const QString& frame_qstr, double speed_ratio) {
    if (!rc_impl_ || !state_data_) return;
    
    TrajectoryPoint jog_cmd;
    jog_cmd.header.motion_type = MotionType::LIN;
    jog_cmd.header.data_type = WaypointDataType::CARTESIAN_DOMINANT_CMD;
    jog_cmd.command.speed_ratio = std::max(0.01, speed_ratio);
    jog_cmd.header.tool = state_data_->getActiveToolFrame();

    TrajectoryPoint fb_sdata = state_data_->getFbTrajectoryPoint();
    CartPose current_tcp_in_world = fb_sdata.feedback.cartesian_actual;
    CartPose target_tcp_in_selected_jog_frame_coords;
    BaseFrame selected_jog_frame;

    if (frame_qstr.compare("Robot Base", Qt::CaseInsensitive) == 0) {
        selected_jog_frame = BaseFrame(CartPose{}, "JogInRobotBase");
        target_tcp_in_selected_jog_frame_coords = current_tcp_in_world;
    } else if (frame_qstr.compare("Active Tool", Qt::CaseInsensitive) == 0) {
        selected_jog_frame.transform = current_tcp_in_world;
        selected_jog_frame.name = "JogInActiveTool";
        target_tcp_in_selected_jog_frame_coords = CartPose{};
    } else { // Active User Base
        selected_jog_frame = state_data_->getActiveBaseFrame();
        target_tcp_in_selected_jog_frame_coords = FrameTransformer::transformPoseFromWorld(current_tcp_in_world, selected_jog_frame.transform);
    }
    
    apply_cartesian_increment(target_tcp_in_selected_jog_frame_coords, axis_idx, delta_val);
    
    jog_cmd.command.cartesian_target = target_tcp_in_selected_jog_frame_coords;
    jog_cmd.header.base = selected_jog_frame;
    
    if (!rc_impl_->executeMotionToTarget(jog_cmd)) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Could not execute cartesian jog.");
    }
}

void Adapter_RobotController::onJogPanelGoHome() {
    if (!rc_impl_) return;
    TrajectoryPoint tp_home;
    tp_home.header.motion_type = MotionType::PTP;
    tp_home.command.joint_target.fromAngleArray({0.0_rad, (-90.0_deg).toRadians(), (90.0_deg).toRadians(), 0.0_rad, 0.0_rad, 0.0_rad});
    tp_home.command.speed_ratio = 0.25;
    if (!rc_impl_->executeMotionToTarget(tp_home)) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Could not execute GoHome.");
    }
}

void Adapter_RobotController::onJogPanelEmergencyStop() {
    if(rc_impl_) rc_impl_->emergencyStop();
}

void Adapter_RobotController::onJogPanelSpeedChanged(int percent) {
    if (state_data_) state_data_->setGlobalSpeedRatio(static_cast<double>(percent) / 100.0);
}

// --- Slots for Status Bar Controls ---
void Adapter_RobotController::onSwitchToSimRequested() {
    if (!rc_impl_) return;
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI: Switch to Simulation requested.");
    (void)rc_impl_->requestModeSwitch(MasterHardwareInterface::ActiveMode::Simulation);
}

void Adapter_RobotController::onSwitchToRealRequested() {
    if (!rc_impl_) return;
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI: Switch to Realtime requested.");
    auto result = rc_impl_->requestModeSwitch(MasterHardwareInterface::ActiveMode::Realtime);

    if (result == RobotController::SwitchRequestResult::NotInSync) {
        LOG_WARN(ADAPTER_MODULE_NAME, "Switch failed: Not in sync. Asking user.");
        QMessageBox::StandardButton reply;
        reply = QMessageBox::question(nullptr, "Synchronization Required", 
            "The simulation and the real robot are not in sync.\n"
            "Do you want to synchronize the simulation to the real robot's position?\n\n"
            "WARNING: This will cause the 3D model to 'jump' to the real robot's position.",
            QMessageBox::Yes|QMessageBox::No);
        
        if (reply == QMessageBox::Yes) {
            LOG_INFO(ADAPTER_MODULE_NAME, "User chose to sync. Forcing sync...");
            rc_impl_->forceSync();
            (void)rc_impl_->requestModeSwitch(MasterHardwareInterface::ActiveMode::Realtime);
        } else {
            LOG_INFO(ADAPTER_MODULE_NAME, "User cancelled sync.");
        }
    }
}

void Adapter_RobotController::onResetErrorRequested() {
    if (!rc_impl_) return;
    LOG_INFO(ADAPTER_MODULE_NAME, "GUI: Reset Error requested.");
    rc_impl_->reset();
}

// --- Helper Functions ---
void Adapter_RobotController::apply_cartesian_increment(CartPose& pose, int axis_idx, double increment) {
    if (axis_idx >= 0 && axis_idx < 3) { // X, Y, Z
        double current_val = pose.get_value_at(axis_idx);
        pose.set_value_at(axis_idx, current_val + Meters(increment / 1000.0).value());
    } else if (axis_idx >= 3 && axis_idx < 6) { // Rx, Ry, Rz
        double current_val = pose.get_value_at(axis_idx);
        pose.set_value_at(axis_idx, current_val + Degrees(increment).toRadians().value());
    }
}

ToolFrame Adapter_RobotController::getToolFrameFromGUIName(const QString& name_qstr) const {
    std::string name = name_qstr.toStdString();
    if (name == "Gripper") return ToolFrame(CartPose{0.0_m, 0.0_m, 0.20_m}, "Gripper");
    if (name == "Welder") return ToolFrame(CartPose{0.05_m, 0.0_m, 0.15_m}, "Welder");
    return ToolFrame(CartPose{}, "DefaultTool");
}

BaseFrame Adapter_RobotController::getBaseFrameFromGUIName(const QString& name_qstr) const {
    std::string name = name_qstr.toStdString();
    if (name == "UserBase1") return BaseFrame(CartPose{0.1_m, 0.1_m, 0.1_m}, "UserBase1");
    return BaseFrame(CartPose{}, "RobotBase");
}

} // namespace RDT