// Panel_StatusBar.cpp
#include "Panel_StatusBar.h"
#include <QPalette>     // For setting background color
#include <QDebug>       // For qDebug (can be replaced by RDT::Logger for panel internal debug)

namespace RDT {

// For convenience
using namespace RDT::literals;

Panel_StatusBar::Panel_StatusBar(QWidget* parent) : QWidget(parent) {
    setupUI();
    // Set initial default display values
    onSetRobotModeRDT(RobotMode::Initializing);
    onSetSystemMessageRDT("System Starting...", false);
    onSetSpeedRatioRDT(1.0); // 100%
    onSetEstopStatusRDT(false);
    onSetConnectionStatusRDT(false);
    LOG_DEBUG(MODULE_NAME, "Panel_StatusBar created and UI set up.");
}

void Panel_StatusBar::setupUI() {
    auto* layout = new QHBoxLayout(this);
    layout->setContentsMargins(8, 4, 8, 4); // Adjusted margins
    layout->setSpacing(15);                 // Adjusted spacing

    QFont labelFont("Arial", 9);
    QFont valueFont("Arial", 9, QFont::Bold);

    mode_label_ = new QLabel("System:", this);
    mode_label_->setFont(labelFont);
    sim_real_mode_selector_ = new QComboBox(this);
    sim_real_mode_selector_->setFont(valueFont);
    sim_real_mode_selector_->addItem("SIMULATOR");
    sim_real_mode_selector_->addItem("REAL ROBOT");
    sim_real_mode_selector_->setToolTip("Select system operation mode (Simulator or Real Hardware)");
    connect(sim_real_mode_selector_, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, [this](int index){
                emit simRealModeChangedByUser(index == 1); // 1 is REAL ROBOT
            });

    robot_state_label_ = new QLabel("State: N/A", this);
    robot_state_label_->setFont(valueFont);
    robot_state_label_->setMinimumWidth(120);

    speed_label_ = new QLabel("Speed: 100%", this);
    speed_label_->setFont(valueFont);
    speed_label_->setMinimumWidth(90);

    connection_label_ = new QLabel("Physically: Disconnected", this);
    connection_label_->setFont(valueFont);
    connection_label_->setMinimumWidth(150);

    system_message_label_ = new QLabel("Status: OK", this); // Was errorLabel_
    system_message_label_->setFont(valueFont);
    system_message_label_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Preferred); // Allow to expand

    layout->addWidget(mode_label_);
    layout->addWidget(sim_real_mode_selector_);
    layout->addSpacing(20);
    layout->addWidget(robot_state_label_);
    layout->addWidget(speed_label_);
    layout->addStretch(1); // Pushes connection and error to the right
    layout->addWidget(connection_label_);
    layout->addSpacing(10);
    layout->addWidget(system_message_label_);

    // Set a distinct background color for the status bar
    setAutoFillBackground(true);
    QPalette pal = palette();
    pal.setColor(QPalette::Window, QColor(Qt::lightGray).lighter(110)); // Slightly lighter gray
    setPalette(pal);
    setFixedHeight(30); // Fixed height
}

// --- Public Slots ---
void Panel_StatusBar::onSetRobotModeRDT(RDT::RobotMode mode) {
    QString modeStr;
    switch (mode) {
        case RobotMode::Idle:         modeStr = "Idle"; break;
        case RobotMode::Running:      modeStr = "Running"; break;
        case RobotMode::Paused:       modeStr = "Paused"; break;
        case RobotMode::EStop:        modeStr = "E-STOPPED"; break; // Distinct for EStop
        case RobotMode::Error:        modeStr = "Error State"; break;
        case RobotMode::Initializing: modeStr = "Initializing"; break;
        case RobotMode::Homing:       modeStr = "Homing"; break;
        case RobotMode::Jogging:      modeStr = "Jogging"; break;
        default:                      modeStr = "Unknown"; break;
    }
    robot_state_label_->setText("State: " + modeStr);
    LOG_DEBUG_F(MODULE_NAME, "RobotMode display updated to: %s", modeStr.toStdString().c_str());

    // If E-Stop mode, override system message to show E-Stop prominently
    if (mode == RobotMode::EStop || estop_is_active_) {
        onSetSystemMessageRDT("EMERGENCY STOP ACTIVE", true);
    }
}

void Panel_StatusBar::onSetSystemMessageRDT(const QString& message, bool isError) {
    if (estop_is_active_ && !message.contains("EMERGENCY STOP", Qt::CaseInsensitive)) {
        // Don't let non-EStop messages clear an active E-Stop message unless it's also an EStop message
        // Or if the error flag is false, indicating EStop might have been cleared
        if (isError) { // A new error message might be more important
             system_message_label_->setText("⚠ " + message);
             system_message_label_->setStyleSheet("color: red; font-weight: bold;");
             LOG_WARN_F(MODULE_NAME, "System message updated to ERROR (EStop active): %s", message.toStdString().c_str());
        } else {
            // Keep EStop message prominent if no new error
        }
        return;
    }

    if (isError) {
        system_message_label_->setText("⚠ " + message);
        system_message_label_->setStyleSheet("color: red; font-weight: bold;");
        LOG_WARN_F(MODULE_NAME, "System message updated to ERROR: %s", message.toStdString().c_str());
    } else {
        system_message_label_->setText("Status: " + (message.isEmpty() ? "OK" : message));
        system_message_label_->setStyleSheet("color: black; font-weight: normal;"); // Reset style
        LOG_INFO_F(MODULE_NAME, "System message updated to STATUS: %s", message.toStdString().c_str());
    }
}

void Panel_StatusBar::onSetSpeedRatioRDT(double ratio) {
    int percent = static_cast<int>(std::round(ratio * 100.0));
    percent = std::max(0, std::min(100, percent)); // Clamp to 0-100
    speed_label_->setText(QString("Speed: %1%").arg(percent));
    LOG_DEBUG_F(MODULE_NAME, "Speed display updated to: %d%%", percent);
}

void Panel_StatusBar::onSetEstopStatusRDT(bool isActive) {
    estop_is_active_ = isActive;
    if (isActive) {
        onSetRobotModeRDT(RobotMode::EStop); // Ensure mode label also reflects EStop
        onSetSystemMessageRDT("EMERGENCY STOPPED", true); // Override message
        // Visual cue for E-Stop, e.g., panel background
        QPalette pal = palette();
        pal.setColor(QPalette::Window, QColorConstants::Svg::orangered);
        setPalette(pal);
    } else {
        // E-Stop cleared, revert visual cues. RobotMode and Message will be updated by regular polling.
        QPalette pal = palette();
        pal.setColor(QPalette::Window, QColor(Qt::lightGray).lighter(110));
        setPalette(pal);
        // If the mode was EStop, it should be updated by a subsequent onSetRobotModeRDT call to Idle/Error etc.
        // If the message was "EMERGENCY STOPPED", it will be cleared by a subsequent onSetSystemMessageRDT.
    }
    LOG_INFO_F(MODULE_NAME, "E-Stop status updated to: %s", isActive ? "ACTIVE" : "INACTIVE");
}

void Panel_StatusBar::onSetConnectionStatusRDT(bool isConnected) {
    connection_label_->setText(isConnected ? "Physically: Connected" : "Physically: Disconnected");
    connection_label_->setStyleSheet(isConnected ? "color: darkgreen; font-weight: bold;" : "color: dimgray;");
    LOG_INFO_F(MODULE_NAME, "Physical connection status display updated to: %s", isConnected ? "Connected" : "Disconnected");
}

void Panel_StatusBar::onSetSimRealModeSelector(bool isReal) {
    sim_real_mode_selector_->blockSignals(true); // Prevent emitting modeChangedByUser
    sim_real_mode_selector_->setCurrentIndex(isReal ? 1 : 0);
    sim_real_mode_selector_->blockSignals(false);
    LOG_INFO_F(MODULE_NAME, "SIM/REAL mode selector set to: %s", isReal ? "REAL" : "SIM");
}


} // namespace RDT