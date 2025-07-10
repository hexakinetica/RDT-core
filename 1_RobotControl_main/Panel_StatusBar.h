// Panel_StatusBar.h
#ifndef PANEL_STATUSBAR_H
#define PANEL_STATUSBAR_H

#pragma once

#include <QWidget>
#include <QLabel>
#include <QComboBox> //   SIM/REAL 
#include <QHBoxLayout>
#include <QString>

//  RDT 
#include "DataTypes.h" // RDT::RobotMode
#include "Logger.h"    // RDT::Logger

namespace RDT {

/**
 * @file Panel_StatusBar.h
 * @brief Defines the RDT::Panel_StatusBar Qt widget for displaying overall system status.
 */

/**
 * @class Panel_StatusBar
 * @brief A Qt widget that displays key status indicators like robot mode, speed,
 *        connection status, and error messages.
 *
 * It receives updates via Qt slots that accept RDT data types or simple types.
 */
class Panel_StatusBar : public QWidget {
    Q_OBJECT

public:
    /**
     * @brief Constructor for Panel_StatusBar.
     * @param parent Optional parent QWidget.
     */
    explicit Panel_StatusBar(QWidget* parent = nullptr);
    ~Panel_StatusBar() override = default;

    // Non-copyable, non-movable
    Panel_StatusBar(const Panel_StatusBar&) = delete;
    Panel_StatusBar& operator=(const Panel_StatusBar&) = delete;
    Panel_StatusBar(Panel_StatusBar&&) = delete;
    Panel_StatusBar& operator=(Panel_StatusBar&&) = delete;

public slots:
    /**
     * @brief Sets the displayed robot operational mode.
     * @param mode The RDT::RobotMode to display.
     */
    void onSetRobotModeRDT(RDT::RobotMode mode);

    /**
     * @brief Sets the system message and error status.
     * If isError is true, the message is displayed prominently as an error.
     * Otherwise, it's displayed as a general status message.
     * @param message The message string.
     * @param isError True if the message indicates an error.
     */
    void onSetSystemMessageRDT(const QString& message, bool isError);

    /**
     * @brief Sets the displayed global speed override as a ratio.
     * @param ratio The speed ratio (0.0 to 1.0).
     */
    void onSetSpeedRatioRDT(double ratio);

    /**
     * @brief Sets the displayed E-Stop status.
     * @param isActive True if E-Stop is active.
     */
    void onSetEstopStatusRDT(bool isActive);

    /**
     * @brief Sets the displayed physical connection status to the robot hardware.
     * @param isConnected True if physically connected.
     */
    void onSetConnectionStatusRDT(bool isConnected);

    /**
     * @brief Slot to set the SIM/REAL mode selector from external logic.
     * @param isReal True to select REAL mode, false for SIM mode.
     */
    void onSetSimRealModeSelector(bool isReal);


signals:
    /**
     * @brief Emitted when the user changes the SIM/REAL mode via the ComboBox.
     * @param isRealMode True if REAL mode is selected, false if SIM mode is selected.
     */
    void simRealModeChangedByUser(bool isRealMode);

private:
    void setupUI(); ///< Initializes and lays out the UI elements.

    // UI Elements
    QLabel* mode_label_ = nullptr;          // "Mode:"
    QComboBox* sim_real_mode_selector_ = nullptr; // ComboBox for SIM/REAL

    QLabel* robot_state_label_ = nullptr;   // Displays RDT::RobotMode as string (e.g., "State: Idle")
    QLabel* speed_label_ = nullptr;         // Displays speed ratio as percent (e.g., "Speed: 75%")
    QLabel* connection_label_ = nullptr;    // Displays "Connected" / "Not Connected"
    QLabel* system_message_label_ = nullptr;// Displays error or status messages

    // Internal state for E-Stop to potentially modify other labels
    bool estop_is_active_ = false;

    static inline const std::string MODULE_NAME = "StatusBarPanel";
};

} // namespace RDT
#endif // PANEL_STATUSBAR_H