// GUI_App_main.cpp
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QTimer>     // QTimer from Adapter uses this
#include <QDebug>
#include <iostream>
#include <memory>     // For std::shared_ptr, std::make_shared, std::make_unique

// --- RDT Core Components ---
#include "DataTypes.h"
#include "Units.h"
#include "Logger.h"
#include "RDT_Qt_MetaTypes.h" // IMPORTANT: For Qt to handle RDT types in signals/slots

#include "StateData.h"
#include "FrameTransformer.h" // Used statically by RobotController/Planner
#include "KinematicModel.h"
#include "KdlKinematicSolver.h"
#include "TrajectoryInterpolator.h"
#include "TrajectoryPlanner.h"
#include "FakeMotionInterface.h"  // Using Fake for this integrated test
#include "MotionManager.h"
#include "RobotController.h"

// --- Adapter & GUI Panels ---
#include "Adapter_RobotController.h" // Our new adapter
#include "Panel_StateDisplay.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"
#include "Panel_RobotView3D.h"


// For convenience
using namespace RDT;
using namespace RDT::literals;
using namespace std::chrono_literals;


int main(int argc, char* argv[]) {
    QApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
    QApplication app(argc, argv);

    // 1. Initialize RDT MetaTypes for Qt and Logger
    RDT::registerRdtMetaTypes();
    RDT::Logger::setLogLevel(RDT::LogLevel::Debug); // Set desired log level

    LOG_INFO("GUI_AppMain", "--- Full GUI Application with RobotController Backend Starting ---");

    // 2. Create Core RDT Components (the "backend")
    LOG_INFO("GUI_AppMain", "Creating RDT core components...");
    auto state_data_ptr = std::make_shared<RDT::StateData>();
    
    RDT::KinematicModel kuka_model = RDT::KinematicModel::createKR6R900();
    auto solver_ptr = std::make_shared<RDT::KdlKinematicSolver>(kuka_model);
    auto interpolator_ptr = std::make_shared<RDT::TrajectoryInterpolator>();
    auto planner_ptr = std::make_shared<RDT::TrajectoryPlanner>(solver_ptr, interpolator_ptr);
    
    unsigned int mm_cycle_period_ms = 100; // MotionManager cycle period
    auto motion_interface_ptr = std::make_unique<RDT::FakeMotionInterface>();
    auto motion_manager_ptr = std::make_shared<RDT::MotionManager>(std::move(motion_interface_ptr), mm_cycle_period_ms);
    
    auto robot_controller_ptr = std::make_shared<RDT::RobotController>(
        planner_ptr, motion_manager_ptr, solver_ptr, state_data_ptr
    );
    LOG_INFO("GUI_AppMain", "RDT core components created.");

    // 3. Create Adapter_RobotController
    LOG_INFO("GUI_AppMain", "Creating Adapter_RobotController...");
    // Adapter_RobotController* adapter = new Adapter_RobotController(robot_controller_ptr, state_data_ptr, &app); // Parented to app
    // Making it a unique_ptr for RAII, but ensure QObject parentage if needed for auto-delete by Qt (e.g. parent to main_window)
    auto adapter_ptr = std::make_unique<RDT::Adapter_RobotController>(robot_controller_ptr, state_data_ptr);
    LOG_INFO("GUI_AppMain", "Adapter_RobotController created.");


    // 4. Create GUI Panels
    LOG_INFO("GUI_AppMain", "Creating GUI panels...");
    auto* teach_panel_logic = new RDT::Panel_Teach(); 
    QWidget* teach_panel_widget = teach_panel_logic->getWidget();
    teach_panel_widget->setMinimumWidth(400);
    teach_panel_widget->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);

    auto* robot_view_panel = new Panel_RobotView3D();
    robot_view_panel->setMinimumSize(700, 500);
    robot_view_panel->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

    auto* jog_panel = new RDT::Panel_JogControl();
    jog_panel->setFixedWidth(300);

    auto* state_display_panel = new RDT::Panel_StateDisplay();
    auto* status_bar_panel = new RDT::Panel_StatusBar();
    status_bar_panel->setFixedHeight(35);
    LOG_INFO("GUI_AppMain", "GUI panels created.");

    // 5. Connect Panels to Adapter
    LOG_INFO("GUI_AppMain", "Connecting panels to Adapter_RobotController...");
    adapter_ptr->connectStateDisplayPanel(state_display_panel);
    adapter_ptr->connectRobotView3D(robot_view_panel);
    adapter_ptr->connectStatusBarPanel(status_bar_panel);
    adapter_ptr->connectTeachPanel(teach_panel_logic); // Pass the logic object
    adapter_ptr->connectJogPanel(jog_panel);
    LOG_INFO("GUI_AppMain", "Panels connected to Adapter.");

    // 6. Assemble Main Window
    LOG_INFO("GUI_AppMain", "Assembling main window...");
    QFrame* top_main_area_frame = new QFrame;
    QGridLayout* top_main_grid_layout = new QGridLayout(top_main_area_frame);
    top_main_grid_layout->setContentsMargins(0,0,0,0); top_main_grid_layout->setSpacing(5);
    top_main_grid_layout->addWidget(teach_panel_widget, 0, 0);
    top_main_grid_layout->addWidget(robot_view_panel, 0, 1);
    top_main_grid_layout->addWidget(jog_panel, 0, 2);
    top_main_grid_layout->setColumnStretch(0, 3); top_main_grid_layout->setColumnStretch(1, 6); top_main_grid_layout->setColumnStretch(2, 2);

    QVBoxLayout* main_vertical_layout = new QVBoxLayout;
    main_vertical_layout->setContentsMargins(5,5,5,5); main_vertical_layout->setSpacing(5);
    main_vertical_layout->addWidget(status_bar_panel);
    main_vertical_layout->addWidget(top_main_area_frame, 1);
    main_vertical_layout->addWidget(state_display_panel);

    QWidget* central_widget = new QWidget;
    central_widget->setLayout(main_vertical_layout);

    QMainWindow main_window;
    main_window.setWindowTitle("Industrial Robot Control GUI - RDT Integrated");
    main_window.setCentralWidget(central_widget);
    main_window.resize(1800, 1000); // Larger window
    main_window.show();
    LOG_INFO("GUI_AppMain", "Main window assembled and shown.");
    
    // Set adapter parent after main_window is created for proper QObject cleanup
    // adapter_ptr->setParent(&main_window); // if adapter_ptr was raw, not needed for unique_ptr if main_window outlives it.

    // 7. Initialize RobotController backend via Adapter and start GUI updates
    LOG_INFO("GUI_AppMain", "Initializing backend via Adapter_RobotController...");
    RDT::TrajectoryPoint initial_user_cmd_for_rc;
    initial_user_cmd_for_rc.header.data_type = RDT::WaypointDataType::JOINT_DOMINANT_CMD;
    std::array<RDT::Radians, RDT::ROBOT_AXES_COUNT> initial_angles = {
       // 0.0_rad, (-90.0_deg).toRadians(), (90.0_deg).toRadians(), 0.0_rad, (90.0_deg).toRadians(), 0.0_rad
        0.0_rad, (0.0_deg).toRadians(), (0.0_deg).toRadians(), 0.0_rad, (0.0_deg).toRadians(), 0.0_rad
    };
    initial_user_cmd_for_rc.command.joint_target.fromAngleArray(initial_angles);
    initial_user_cmd_for_rc.header.tool = RDT::ToolFrame(RDT::CartPose{0.0_m, 0.0_m, 0.15_m}, "InitTool");
    initial_user_cmd_for_rc.header.base = RDT::BaseFrame(RDT::CartPose{}, "InitBase"); // T_robotBase_User = Identity

    if (adapter_ptr->initializeBackendAndStartGUIUpdates(initial_user_cmd_for_rc, 100 /*ms update interval*/)) {
        LOG_INFO("GUI_AppMain", "Backend initialized successfully via Adapter.");
        // Load 3D model after backend init (which starts MM, which might start interface)
        if (robot_view_panel->loadmodel()) { // Path is hardcoded in Panel_RobotView3D for now
             LOG_INFO("GUI_AppMain", "Robot 3D model loaded.");
        } else {
            LOG_WARN("GUI_AppMain", "Failed to load robot 3D model.");
        }
    } else {
        LOG_CRITICAL("GUI_AppMain", "Backend initialization FAILED via Adapter. GUI may not function correctly.");
        // Handle critical error, maybe close app or show error dialog
    }

    LOG_INFO("GUI_AppMain", "GUI event loop started. Close main window to exit.");
    int result = app.exec();

    LOG_INFO("GUI_AppMain", "GUI Application Shutting Down...");
    // RobotController's jthread will be joined in its destructor when adapter_ptr (and then robot_controller_ptr) go out of scope.
    // Qt will handle deletion of QWidgets based on parent-child relationships.
    // delete teach_panel_logic; // Only if it was not parented to a QWidget that Qt manages.
                               // If teach_panel_widget was parented, and teach_panel_logic is QObject parented
                               // to teach_panel_widget, it should be fine.
                               // Or, if teach_panel_logic is parented to adapter_ptr, and adapter_ptr is parented to main_window.

    LOG_INFO("GUI_AppMain", "GUI Application End.");
    return result;
}