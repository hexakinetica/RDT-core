// Panel_StatusBar.cpp
#include "Panel_StatusBar.h"
#include <QPalette>

namespace RDT {

Panel_StatusBar::Panel_StatusBar(QWidget* parent) : QWidget(parent) {
    setupUI();
    // Set initial values
    onSetRobotModeRDT(RobotMode::Initializing);
    onSetSystemMessageRDT("System Starting...", false);
    onSetSpeedRatioRDT(1.0);
    onSetEstopStatusRDT(false);
    onSetConnectionStatusRDT(false);
    onActiveModeChanged("Simulation", false); // Initial mode
}

void Panel_StatusBar::setupUI() {
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(8, 4, 8, 4);
    layout->setSpacing(10);

    QFont valueFont("Arial", 9, QFont::Bold);

    // --- Mode Control Elements ---
    active_mode_label_ = new QLabel("Mode: SIMULATION", this);
    active_mode_label_->setFont(valueFont);
    
    // This button is a placeholder for a future connection sequence
    btn_connect_real_ = new QPushButton("Connect Real", this);
    btn_connect_real_->setToolTip("Attempt to connect to the real hardware interface.");
    btn_connect_real_->setEnabled(false); // Disabled for this implementation
    
    btn_switch_to_sim_ = new QPushButton("Switch to Sim", this);
    btn_switch_to_sim_->setToolTip("Switch control to the internal simulation.");

    btn_switch_to_real_ = new QPushButton("Switch to Real", this);
    btn_switch_to_real_->setToolTip("Switch control to the real hardware (requires sync).");

    btn_reset_error_ = new QPushButton("Reset Error", this);
    btn_reset_error_->setToolTip("Resets the controller from an error state.");

    connect(btn_switch_to_sim_, &QPushButton::clicked, this, &Panel_StatusBar::switchToSimRequested);
    connect(btn_switch_to_real_, &QPushButton::clicked, this, &Panel_StatusBar::switchToRealRequested);
    connect(btn_reset_error_, &QPushButton::clicked, this, &Panel_StatusBar::resetErrorRequested);

    layout->addWidget(active_mode_label_);
    layout->addWidget(btn_switch_to_sim_);
    layout->addWidget(btn_switch_to_real_);
    layout->addWidget(btn_reset_error_);
    layout->addSpacing(20);

    // --- State Indicators ---
    robot_state_label_ = new QLabel("State: N/A", this);
    robot_state_label_->setFont(valueFont);
    robot_state_label_->setMinimumWidth(120);

    speed_label_ = new QLabel("Speed: 100%", this);
    speed_label_->setFont(valueFont);
    speed_label_->setMinimumWidth(90);

    connection_label_ = new QLabel("HW: Disconnected", this);
    connection_label_->setFont(valueFont);
    connection_label_->setMinimumWidth(150);

    system_message_label_ = new QLabel("Status: OK", this);
    system_message_label_->setFont(valueFont);
    system_message_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred);

    layout->addWidget(robot_state_label_);
    layout->addWidget(speed_label_);
    layout->addStretch(1);
    layout->addWidget(connection_label_);
    layout->addWidget(system_message_label_);

    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(Qt::lightGray).lighter(110));
    setPalette(pal);
    setFixedHeight(35);
}

void Panel_StatusBar::onActiveModeChanged(const QString& modeName, bool isRealConnected) {
    active_mode_label_->setText("Mode: " + modeName.toUpper());
    
    // Button enable/disable logic
    btn_switch_to_sim_->setEnabled(modeName.compare("Simulation", Qt::CaseInsensitive) != 0);
    btn_switch_to_real_->setEnabled(modeName.compare("Realtime", Qt::CaseInsensitive) != 0 && isRealConnected);
}

void Panel_StatusBar::onSetRobotModeRDT(RDT::RobotMode mode) {
    QString modeStr;
    bool isErrorState = false;
    switch (mode) {
        case RobotMode::Idle:         modeStr = "Idle"; break;
        case RobotMode::Running:      modeStr = "Running"; break;
        case RobotMode::Paused:       modeStr = "Paused"; break;
        case RobotMode::EStop:        modeStr = "E-STOPPED"; isErrorState = true; break;
        case RobotMode::Error:        modeStr = "Error State"; isErrorState = true; break;
        case RobotMode::Initializing: modeStr = "Initializing"; break;
        case RobotMode::Homing:       modeStr = "Homing"; break;
        case RobotMode::Jogging:      modeStr = "Jogging"; break;
        default:                      modeStr = "Unknown"; break;
    }
    robot_state_label_->setText("State: " + modeStr);
    
    // Update reset button state based on robot mode
    error_is_active_ = isErrorState;
    btn_reset_error_->setEnabled(error_is_active_);
}

void Panel_StatusBar::onSetSystemMessageRDT(const QString& message, bool isError) {
    // Update reset button state if an error message is received
    error_is_active_ = isError;
    btn_reset_error_->setEnabled(error_is_active_);

    if (isError || estop_is_active_) {
        system_message_label_->setText("⚠ " + message);
        system_message_label_->setStyleSheet("color: red; font-weight: bold;");
    } else {
        system_message_label_->setText("Status: " + (message.isEmpty() ? "OK" : message));
        system_message_label_->setStyleSheet("color: black; font-weight: normal;");
    }
}

void Panel_StatusBar::onSetSpeedRatioRDT(double ratio) {
    int percent = static_cast<int>(std::round(ratio * 100.0));
    speed_label_->setText(QString("Speed: %1%").arg(percent));
}

void Panel_StatusBar::onSetEstopStatusRDT(bool isActive) {
    estop_is_active_ = isActive;
    QPalette pal = palette();
    if (isActive) {
        pal.setColor(QPalette::Window, QColorConstants::Svg::orangered);
        onSetSystemMessageRDT("EMERGENCY STOP ACTIVE", true);
    } else {
        pal.setColor(QPalette::Window, QColor(Qt::lightGray).lighter(110));
        // We don't clear the system message here, as another error might be present.
        // The next call to onSetSystemMessageRDT will handle it.
    }
    setPalette(pal);
}

void Panel_StatusBar::onSetConnectionStatusRDT(bool isConnected) {
    connection_label_->setText(isConnected ? "HW: Connected" : "HW: Disconnected");
    connection_label_->setStyleSheet(isConnected ? "color: darkgreen; font-weight: bold;" : "color: dimgray;");
}

} // namespace RDT