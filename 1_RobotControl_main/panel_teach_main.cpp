// panel_teach_main.cpp
#include <QApplication>
#include <QMainWindow> //  QMainWindow   
#include <QVBoxLayout>
#include <QDebug>
#include <QTimer>

#include "Panel_Teach.h"        //  
#include "Mock_Adapter.h" //  -
#include "DataTypes.h"          // RDT::TrajectoryPoint, etc.
#include "Units.h"              // RDT::literals
#include "Logger.h"             // RDT::Logger
#include "RDT_Qt_MetaTypes.h"   //   

//  
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;


int main(int argc, char *argv[]) {
    QApplication app(argc, argv);

    RDT::registerRdtMetaTypes();
    Logger::setLogLevel(LogLevel::Debug);

    LOG_INFO("TeachPanelTest", "--- Panel_Teach Test Application (with Mock Adapter) ---");

    //      
    Panel_Teach teachPanelLogic; 
    QWidget* teachPanelWidget = teachPanelLogic.getWidget();

    //  -     
    Mock_Adapter mock_adapter(&teachPanelLogic);

    //       -,
    //     
    QVector<RDT::TrajectoryPoint> initial_program;
    TrajectoryPoint p1;
    p1.header.motion_type = MotionType::PTP;
    p1.command.joint_target[AxisId::A1].angle = (10.0_deg).toRadians();
    p1.command.joint_target[AxisId::A2].angle = (20.0_deg).toRadians();
    p1.command.cartesian_target.x = 0.1_m; // 
    p1.header.tool.name = "DefaultTool"; 
    p1.header.base.name = "RobotBase"; 
    p1.command.speed_ratio = 1.0;
    initial_program.append(p1);

    TrajectoryPoint p2 = p1; // 
    p2.header.motion_type = MotionType::LIN;
    p2.command.joint_target[AxisId::A1].angle = (15.0_deg).toRadians(); //  
    p2.command.cartesian_target.x = 0.15_m;
    p2.command.cartesian_target.y = 0.05_m;
    p2.header.tool.name = "Gripper"; 
    p2.header.base.name = "UserBase1"; 
    p2.command.speed_ratio = 0.5;
    initial_program.append(p2);
    
    mock_adapter.setInitialProgram(initial_program); //      


    // --- Setup Window ---
    QMainWindow mainWindow; 
    mainWindow.setWindowTitle("Panel_Teach Test (RDT Types & Mock Adapter)");
    mainWindow.setCentralWidget(teachPanelWidget); 
    mainWindow.resize(600, 800); //  
    mainWindow.show();

    //       ( -)
    QTimer::singleShot(2000, [&mock_adapter]() {
        LOG_INFO("TeachPanelTest", "Simulating highlight step 0 via mock adapter.");
        mock_adapter.simulateHighlightStep(0);
    });
    QTimer::singleShot(4000, [&mock_adapter]() {
        LOG_INFO("TeachPanelTest", "Simulating highlight step 1 via mock adapter.");
        mock_adapter.simulateHighlightStep(1);
    });
     QTimer::singleShot(6000, [&mock_adapter]() {
        LOG_INFO("TeachPanelTest", "Simulating highlight clear via mock adapter.");
        mock_adapter.simulateHighlightStep(-1); //  
    });


    LOG_INFO("TeachPanelTest", "Test window shown. Event loop started.");
    return app.exec();
}