// panel_statedisplay_main.cpp
#include <QApplication>
#include <QVBoxLayout>
#include <QPushButton>
#include <QTimer>      // For simulating updates
#include <QDebug>      // For qDebug

#include "Panel_StateDisplay.h" // Our refactored panel
#include "DataTypes.h"          // RDT::CartPose, AxisSet, ToolFrame, BaseFrame
#include "Units.h"              // RDT::literals
#include "Logger.h"             // RDT::Logger
#include "RDT_Qt_MetaTypes.h"   // For qRegisterMetaType (needed if signals pass RDT types)

// For convenience
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;




int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    // IMPORTANT: Register RDT MetaTypes for Qt's signal/slot system if passing RDT objects
    RDT::registerRdtMetaTypes(); // Call our registration function
    Logger::setLogLevel(LogLevel::Debug);

    LOG_INFO("StatePanelTest", "--- Panel_StateDisplay Test Application ---");

    QWidget window;
    QVBoxLayout *layout = new QVBoxLayout(&window);

    Panel_StateDisplay* statePanel = new Panel_StateDisplay(); // Panel is now in RDT namespace
    layout->addWidget(statePanel);

    // Create a mock adapter to emit signals
    MoMckRobotControllerAdapter mock_adapter;

    // Connect signals from mock adapter to slots in the panel
    QObject::connect(&mock_adapter, &MoMckRobotControllerAdapter::currentPoseUpdated,
                     statePanel, &Panel_StateDisplay::onCurrentPoseUpdate);
    QObject::connect(&mock_adapter, &MoMckRobotControllerAdapter::activeToolFrameChanged,
                     statePanel, &Panel_StateDisplay::onActiveToolUpdate);
    QObject::connect(&mock_adapter, &MoMckRobotControllerAdapter::activeBaseFrameChanged,
                     statePanel, &Panel_StateDisplay::onActiveBaseUpdate);


    // --- Test Data & Buttons ---
    CartPose test_tcp_pose = {(100.0_mm).toMeters(), (-50.0_mm).toMeters(),  (300.0_mm).toMeters(),
                              0.1_rad, 0.2_rad, 1.57_rad}; // Initial TCP
    std::array<Radians, ROBOT_AXES_COUNT> initial_angles_rad = {
        0.1_rad, -0.2_rad, 0.3_rad, -0.4_rad, 0.5_rad, -0.6_rad
    };
    AxisSet test_joint_set;
    test_joint_set.fromAngleArray(initial_angles_rad); // Initial joints

    ToolFrame test_tool("InitialTestTool", CartPose{0.0_m,0.0_m,0.1_m}); // Tool T_flange_TCP
    BaseFrame test_base("InitialWorldBase", CartPose{});             // Base T_robotBase_UserBase (identity)


    // Button to simulate updating TCP and Joints
    QPushButton *btnUpdatePose = new QPushButton("Update Pose & Joints");
    QObject::connect(btnUpdatePose, &QPushButton::clicked, [&]() {
        static double x_mm_val = 100.0;
        test_tcp_pose.x = Meters(x_mm_val / 1000.0); // Convert mm to m for RDT::Meters
        x_mm_val += 25.0; // Increment by 25mm
        test_tcp_pose.y = Millimeters(-50.0 + (rand() % 1000 - 500) / 10.0).toMeters(); // Random Y in mm, then to m
        test_tcp_pose.rz = (test_tcp_pose.rz + (5.0_deg).toRadians()).normalized(); // Increment Rz by 5 deg

        for(size_t i = 0; i < ROBOT_AXES_COUNT; ++i) {
            test_joint_set[static_cast<AxisId>(i)].angle = Radians(( (rand() % 3600) - 1800 ) / 10.0).toDegrees().toRadians().normalized(); // Random angle
        }
        
        emit mock_adapter.currentPoseUpdated(test_tcp_pose, test_joint_set);
        LOG_DEBUG_F("StatePanelTest", "Emitted currentPoseUpdated: TCP.X %s, J1 %s",
                    test_tcp_pose.x.toString().c_str(), test_joint_set[AxisId::A1].angle.toDegrees().toString().c_str());
    });
    layout->addWidget(btnUpdatePose);

    // Button to simulate changing Tool
    QPushButton *btnChangeTool = new QPushButton("Change Tool");
    QObject::connect(btnChangeTool, &QPushButton::clicked, [&]() {
        static int tool_idx = 0;
        tool_idx++;
        if (tool_idx % 2 == 0) {
            test_tool = ToolFrame(CartPose{0.0_m, 0.0_m, 0.15_m}, "WeldingGun");
        } else {
            test_tool = ToolFrame(CartPose{0.02_m, 0.0_m, 0.05_m}, "GripperV2");
        }
        emit mock_adapter.activeToolFrameChanged(test_tool);
        LOG_DEBUG_F("StatePanelTest", "Emitted activeToolFrameChanged: %s", test_tool.name.c_str());
    });
    layout->addWidget(btnChangeTool);

    // Button to simulate changing Base
    QPushButton *btnChangeBase = new QPushButton("Change Base");
    QObject::connect(btnChangeBase, &QPushButton::clicked, [&]() {
        static int base_idx = 0;
        base_idx++;
        if (base_idx % 2 == 0) {
            test_base = BaseFrame(CartPose{0.5_m, 0.0_m, 0.1_m, 0.0_rad, 0.0_rad, (15.0_deg).toRadians()}, "WorkpieceFixture");
        } else {
            test_base = BaseFrame(CartPose{}, "RobotOrigin"); // Identity
        }
        emit mock_adapter.activeBaseFrameChanged(test_base);
        LOG_DEBUG_F("StatePanelTest", "Emitted activeBaseFrameChanged: %s", test_base.name.c_str());
    });
    layout->addWidget(btnChangeBase);


    // Initial signals to populate panel
    QTimer::singleShot(50, [&](){ // Emit after event loop starts
        LOG_INFO("StatePanelTest", "Emitting initial state to panel...");
        emit mock_adapter.activeToolFrameChanged(test_tool);
        emit mock_adapter.activeBaseFrameChanged(test_base);
        emit mock_adapter.currentPoseUpdated(test_tcp_pose, test_joint_set);
    });


    window.setWindowTitle("Panel_StateDisplay Test (RDT Types)");
    window.resize(750, 280); 
    window.show();

    LOG_INFO("StatePanelTest", "Test window shown. Event loop started.");
    return app.exec();
}