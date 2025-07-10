// panel_statusbar_main.cpp
#include <QApplication>
#include <QVBoxLayout>
#include <QWidget>
#include <QPushButton>
#include <QTimer>
#include <QDebug>

#include "Panel_StatusBar.h"    // Our refactored panel
#include "DataTypes.h"          // RDT::RobotMode
#include "Units.h"              // RDT::literals
#include "Logger.h"             // RDT::Logger
#include "RDT_Qt_MetaTypes.h"   // For qRegisterMetaType
#include "Mock_Adapter.h" // Mock adapter to simulate signals
// For convenience
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    RDT::registerRdtMetaTypes();
    Logger::setLogLevel(LogLevel::Debug);

    LOG_INFO("StatusBarTest", "--- Panel_StatusBar Test Application ---");

    QWidget window;
    window.setWindowTitle("Panel_StatusBar Test");
    QVBoxLayout *layout = new QVBoxLayout(&window);

    Panel_StatusBar* statusBar = new Panel_StatusBar(); // Panel is in RDT namespace
    layout->addWidget(statusBar);

    Mock_Adapter mock_adapter;

    // Connect signals from mock adapter to slots in the panel
    QObject::connect(&mock_adapter, &Mock_Adapter::robotModeChanged,
                     statusBar, &Panel_StatusBar::onSetRobotModeRDT);
    QObject::connect(&mock_adapter, &Mock_Adapter::systemMessageUpdated,
                     statusBar, &Panel_StatusBar::onSetSystemMessageRDT);
    QObject::connect(&mock_adapter, &Mock_Adapter::speedRatioChanged,
                     statusBar, &Panel_StatusBar::onSetSpeedRatioRDT);
    QObject::connect(&mock_adapter, &Mock_Adapter::estopStateChanged,
                     statusBar, &Panel_StatusBar::onSetEstopStatusRDT);
    QObject::connect(&mock_adapter, &Mock_Adapter::physicalConnectionChanged,
                     statusBar, &Panel_StatusBar::onSetConnectionStatusRDT);
    
    // Connect signal from panel's ComboBox back to a lambda for logging
    QObject::connect(statusBar, &Panel_StatusBar::simRealModeChangedByUser,
        [](bool isReal){
            LOG_INFO_F("StatusBarTest", "User changed SIM/REAL mode to: %s", isReal ? "REAL" : "SIM");
            // In a real app, this would go to Adapter_RobotController to switch backend
        });


    // --- Test Buttons ---
    QPushButton *btnIdle = new QPushButton("Set Mode: Idle");
    QObject::connect(btnIdle, &QPushButton::clicked, [&]() {
        emit mock_adapter.robotModeChanged(RobotMode::Idle);
        emit mock_adapter.systemMessageUpdated("System Idle and Ready.", false);
    });
    layout->addWidget(btnIdle);

    QPushButton *btnRunning = new QPushButton("Set Mode: Running");
    QObject::connect(btnRunning, &QPushButton::clicked, [&]() {
        emit mock_adapter.robotModeChanged(RobotMode::Running);
        emit mock_adapter.systemMessageUpdated("Program ABC executing.", false);
    });
    layout->addWidget(btnRunning);

    QPushButton *btnError = new QPushButton("Set System Error");
    QObject::connect(btnError, &QPushButton::clicked, [&]() {
        // Mode might go to Error via a different signal or logic,
        // here we just set an error message.
        emit mock_adapter.systemMessageUpdated("E-123: Servo Overload Axis 3", true);
        // emit mock_adapter.robotModeChanged(RobotMode::Error); // Optionally
    });
    layout->addWidget(btnError);
    
    QPushButton *btnEStop = new QPushButton("TOGGLE E-STOP");
    QObject::connect(btnEStop, &QPushButton::clicked, [&]() {
        static bool estop = false;
        estop = !estop;
        emit mock_adapter.estopStateChanged(estop);
        // RobotMode will be updated by the panel's estop slot
    });
    layout->addWidget(btnEStop);

    QPushButton *btnSpeed = new QPushButton("Set Speed 50%");
    QObject::connect(btnSpeed, &QPushButton::clicked, [&]() {
        emit mock_adapter.speedRatioChanged(0.5);
    });
    layout->addWidget(btnSpeed);

    QPushButton *btnConnection = new QPushButton("Toggle Connection");
    QObject::connect(btnConnection, &QPushButton::clicked, [&]() {
        static bool connected = false;
        connected = !connected;
        emit mock_adapter.physicalConnectionChanged(connected);
    });
    layout->addWidget(btnConnection);
    
    QPushButton *btnSetSim = new QPushButton("Set Mode Selector: SIM");
    QObject::connect(btnSetSim, &QPushButton::clicked, statusBar, [statusBar](){
        statusBar->onSetSimRealModeSelector(false);
    });
    layout->addWidget(btnSetSim);


    // Initial state via signals after a short delay
    QTimer::singleShot(100, [&](){
        LOG_INFO("StatusBarTest", "Emitting initial state to panel via mock adapter...");
        emit mock_adapter.robotModeChanged(RobotMode::Initializing);
        emit mock_adapter.systemMessageUpdated("System Initializing...", false);
        emit mock_adapter.speedRatioChanged(1.0);
        emit mock_adapter.estopStateChanged(false);
        emit mock_adapter.physicalConnectionChanged(false);
        statusBar->onSetSimRealModeSelector(false); // Set initial ComboBox state
    });

    window.setLayout(layout);
    window.resize(800, 150);
    window.show();

    LOG_INFO("StatusBarTest", "Test window shown. Event loop started.");
    return app.exec();
}