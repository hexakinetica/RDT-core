// Panel_StatusBar.h
#ifndef PANEL_STATUSBAR_H
#define PANEL_STATUSBAR_H

#pragma once

#include <QWidget>
#include <QLabel>
#include <QPushButton>
#include <QHBoxLayout>
#include <QString>

#include "DataTypes.h"
#include "Logger.h"

namespace RDT {

/**
 * @class Panel_StatusBar
 * @brief A widget for displaying the robot's overall status and providing top-level controls.
 *
 * This panel shows the robot's mode, speed, connection status, and system messages.
 * It also provides buttons for switching between Simulation and Realtime modes,
 * and for resetting system errors.
 */
class Panel_StatusBar : public QWidget {
    Q_OBJECT

public:
    explicit Panel_StatusBar(QWidget* parent = nullptr);
    ~Panel_StatusBar() override = default;

    Panel_StatusBar(const Panel_StatusBar&) = delete;
    Panel_StatusBar& operator=(const Panel_StatusBar&) = delete;

public slots:
    /** @brief Updates the displayed robot mode (e.g., Idle, Running, Error). */
    void onSetRobotModeRDT(RDT::RobotMode mode);
    /** @brief Updates the system message and its color based on error state. */
    void onSetSystemMessageRDT(const QString& message, bool isError);
    /** @brief Updates the displayed speed override percentage. */
    void onSetSpeedRatioRDT(double ratio);
    /** @brief Updates the UI to reflect E-Stop status. */
    void onSetEstopStatusRDT(bool isActive);
    /** @brief Updates the hardware connection status indicator. */
    void onSetConnectionStatusRDT(bool isConnected);
    
    /** 
     * @brief Updates the mode display ("SIMULATION" / "REALTIME") and enables/disables control buttons.
     * @param modeName The name of the active mode.
     * @param isRealConnected True if the real hardware interface is connected and available.
     */
    void onActiveModeChanged(const QString& modeName, bool isRealConnected);

signals:
    // --- Signals for Mode & State Control ---
    /** @brief Emitted when the user requests to connect to the real robot hardware. */
    void connectRealRobotRequested(); // Placeholder for now
    /** @brief Emitted when the user requests to switch to Simulation mode. */
    void switchToSimRequested();
    /** @brief Emitted when the user requests to switch to Realtime mode. */
    void switchToRealRequested();
    /** @brief Emitted when the user requests to reset a system error. */
    void resetErrorRequested();

private:
    void setupUI();

    // --- State Indicators ---
    QLabel* robot_state_label_ = nullptr;
    QLabel* speed_label_ = nullptr;
    QLabel* connection_label_ = nullptr;
    QLabel* system_message_label_ = nullptr;

    // --- Mode Controls ---
    QLabel* active_mode_label_ = nullptr;
    QPushButton* btn_connect_real_ = nullptr;
    QPushButton* btn_switch_to_sim_ = nullptr;
    QPushButton* btn_switch_to_real_ = nullptr;
    QPushButton* btn_reset_error_ = nullptr;

    bool estop_is_active_ = false; ///< Local cache of E-Stop state for UI logic.
    bool error_is_active_ = false; ///< Local cache of error state for UI logic.

    static inline const std::string MODULE_NAME = "StatusBarPanel";
};

} // namespace RDT
#endif // PANEL_STATUSBAR_H