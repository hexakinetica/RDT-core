// Mock_Adapter.h
#ifndef Mock_Adapter_H
#define Mock_Adapter_H

#pragma once

#include <QObject>
#include <QVector>
#include <QString>
#include <thread> //  std::this_thread::sleep_for

#include "DataTypes.h" // RDT::TrajectoryPoint, RDT::MotionType
#include "Units.h"     // RDT::literals
#include "Logger.h"    // RDT::Logger
#include "Panel_Teach.h" // RDT::Panel_Teach ( )

namespace RDT {

/**
 * @class Mock_Adapter
 * @brief A mock Qt adapter class for testing Panel_Teach in isolation.
 *
 * This class simulates the signals and slots that Panel_Teach would interact with
 * from a real RobotControllerAdapter. It maintains a mock program list and logs
 * received signals.
 */
class Mock_Adapter : public QObject {
    Q_OBJECT

public:
    explicit Mock_Adapter(Panel_Teach* panel_to_connect, QObject* parent = nullptr);
    ~Mock_Adapter() override = default;

    // Public method to allow test main to populate initial program
    void setInitialProgram(const QVector<RDT::TrajectoryPoint>& program);


public slots:
    // Slots that Panel_Teach's signals will connect to
    void onGuiTeachCurrentPoseRequested(RDT::MotionType type, const QString& toolName, const QString& baseName, double speedRatio);
    void onGuiDeleteProgramPointRequested(int index);
    void onGuiTouchUpProgramPointRequested(int index, RDT::MotionType type, const QString& newToolName, const QString& newBaseName, double speedRatio);
    void onGuiRunProgramExecutionRequested();
    void onGuiPauseResumeProgramExecutionRequested();
    void onGuiStopProgramExecutionRequested();
    void onGuiProgramSpeedOverrideChanged(int speed_percent);
    void onGuiActiveToolSelectionChanged(const QString& toolName);
    void onGuiActiveBaseSelectionChanged(const QString& baseName);

    // Mock method to simulate highlighting a step (called by test, emits to panel)
    void simulateHighlightStep(int index);

signals:
    // Signals that Panel_Teach's slots might connect to (if it needed to receive data back)
    // For this mock, we mostly care about Panel_Teach -> Adapter signals.
    // But if Panel_Teach had slots to update its display based on adapter signals, they'd be here.
    // For example, if RobotController confirmed a point was added or an error occurred.
    void programWasUpdatedSignal(); // Simple signal to tell panel to refresh from this mock's data
    void highlightStepInPanelSignal(int index);

    void currentPoseUpdated(const RDT::CartPose& tcp_in_current_base, const RDT::AxisSet& joints); //state display
    void activeToolFrameChanged(const RDT::ToolFrame& toolFrame);
    void activeBaseFrameChanged(const RDT::BaseFrame& baseFrame);
    // Add other signals if Panel_StateDisplay subscribes to them

    void robotModeChanged(RDT::RobotMode mode);                       //status bar
    void systemMessageUpdated(const QString& message, bool isError);
    void speedRatioChanged(double ratio);
    void estopStateChanged(bool isActive);
    void physicalConnectionChanged(bool isConnected);

    void currentJointsChanged(const RDT::AxisSet& joints);


     //void currentJointsChanged(const RDT::AxisSet& joints);
private:
    QVector<RDT::TrajectoryPoint> program_data_mock_; ///< Mock storage for the program.
    Panel_Teach* teach_panel_ptr_;                    ///< Pointer to the Panel_Teach instance to update its display.

    //     QString  RDT::ToolFrame/BaseFrame
    // (     /,     RobotController  )
    //      .
    RDT::ToolFrame getToolFrameByName(const QString& name) const;
    RDT::BaseFrame getBaseFrameByName(const QString& name) const;

    static inline const std::string MODULE_NAME = "MockTeachAdapter";
};

} // namespace RDT
#endif // Mock_Adapter_H