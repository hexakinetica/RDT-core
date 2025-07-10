// panel_jogcontrol_main.cpp
#include <QApplication>
#include <QVBoxLayout>
#include <QDebug> // For qDebug

#include "Panel_JogControl.h"   // Our refactored panel
#include "Logger.h"             // RDT::Logger
#include "DataTypes.h"          // For RDT::AxisId (used in connect for context)
#include "RDT_Qt_MetaTypes.h"   // For qRegisterMetaType (if any RDT types were in signals)

// For convenience
using namespace RDT;
// using namespace RDT::literals; // Not strictly needed here as signals use primitives
using namespace std::chrono_literals;

int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    RDT::registerRdtMetaTypes(); // Register RDT types, though JogPanel signals mostly use primitives
    Logger::setLogLevel(LogLevel::Debug);

    LOG_INFO("JogPanelTest", "--- Panel_JogControl Test Application ---");

    Panel_JogControl jogPanel; // Panel is in RDT namespace

    // Connect signals to lambdas for logging/verification
    QObject::connect(&jogPanel, &Panel_JogControl::jogIncrementJointRequested,
        [](int axis_idx, double delta_rad, double speed_ratio) {
            LOG_INFO_F("JogTestMain", "Signal jogIncrementJoint: AxisIdx=%d, Delta=%.4f rad, SpeedRatio=%.2f",
                       axis_idx, delta_rad, speed_ratio);
            // Example: Convert AxisId back for logging
            if (axis_idx >=0 && axis_idx < static_cast<int>(ROBOT_AXES_COUNT)) {
                 LOG_DEBUG_F("JogTestMain", "  (Corresponds to AxisId: %s)", RDT::to_string(static_cast<AxisId>(axis_idx)).c_str());
            }
        });

    QObject::connect(&jogPanel, &Panel_JogControl::jogIncrementCartesianRequested,
        [](int axis_idx, double delta_mm_or_rad, const QString& frame_name, double speed_ratio) {
            LOG_INFO_F("JogTestMain", "Signal jogIncrementCartesian: AxisIdx=%d, Delta=%.4f (mm or rad), Frame='%s', SpeedRatio=%.2f",
                       axis_idx, delta_mm_or_rad, frame_name.toStdString().c_str(), speed_ratio);
        });

    QObject::connect(&jogPanel, &Panel_JogControl::jogContinuousJointStartRequested,
        [](int axis_idx, bool positive_dir, double speed_ratio) {
            LOG_INFO_F("JogTestMain", "Signal jogContinuousJointStart: AxisIdx=%d, Dir=%s, SpeedRatio=%.2f",
                       axis_idx, (positive_dir ? "+" : "-"), speed_ratio);
        });
    QObject::connect(&jogPanel, &Panel_JogControl::jogContinuousCartesianStartRequested,
        [](int axis_idx, bool positive_dir, const QString& frame_name, double speed_ratio) {
            LOG_INFO_F("JogTestMain", "Signal jogContinuousCartesianStart: AxisIdx=%d, Dir=%s, Frame='%s', SpeedRatio=%.2f",
                       axis_idx, (positive_dir ? "+" : "-"), frame_name.toStdString().c_str(), speed_ratio);
        });
    
        
    QObject::connect(&jogPanel, &Panel_JogControl::jogContinuousJointStopRequested,
        [](int axis_idx) {
            LOG_INFO_F("JogTestMain", "Signal jogContinuousJointStop: AxisIdx=%d", axis_idx);
        }); // Corrected signal name
    QObject::connect(&jogPanel, &Panel_JogControl::jogContinuousCartesianStopRequested,
        [](int axis_idx, const QString& frame_name) {
            LOG_INFO_F("JogTestMain", "Signal jogContinuousCartesianStop: AxisIdx=%d, Frame='%s'",
                       axis_idx, frame_name.toStdString().c_str());
        });


    QObject::connect(&jogPanel, &Panel_JogControl::goHomeRequested,
                     []() { LOG_INFO("JogTestMain", "Signal goHomeRequested"); });
    QObject::connect(&jogPanel, &Panel_JogControl::emergencyStopRequested,
                     []() { LOG_CRITICAL("JogTestMain", "Signal emergencyStopRequested"); });
    QObject::connect(&jogPanel, &Panel_JogControl::jogSpeedChanged,
                     [](int percent) { LOG_INFO_F("JogTestMain", "Signal jogSpeedChanged: %d %%", percent); });


    // --- Setup Window ---
    QWidget window;
    QVBoxLayout *mainWinLayout = new QVBoxLayout(&window);
    mainWinLayout->addWidget(&jogPanel);
    
    window.setWindowTitle("Panel_JogControl Test (RDT Types)");
    window.setMinimumSize(320, 650); // Adjusted size
    window.show();

    LOG_INFO("JogPanelTest", "Test window shown. Event loop started.");
    return app.exec();
}