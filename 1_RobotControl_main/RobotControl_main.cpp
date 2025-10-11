// RobotControl_main.cpp
#include <QApplication>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFrame>
#include <QMessageBox>
#include <QTimer>
#include <QDebug>
#include <iostream>
#include <memory>

// --- RDT Core Components ---
#include "DataTypes.h"
#include "Units.h"
#include "Logger.h"
#include "RDT_Qt_MetaTypes.h"
#include "StateData.h"
#include "KinematicModel.h"
#include "RobotConfig.h"
#include "RobotController.h"

// --- Adapter & GUI Panels ---
#include "Adapter_RobotController.h"
#include "Panel_StateDisplay.h"
#include "Panel_StatusBar.h"
#include "Panel_Teach.h"
#include "Panel_JogControl.h"
#include "Panel_RobotView3D.h"

using namespace RDT;
using namespace RDT::literals;

int main(int argc, char* argv[]) {
    QApplication::setAttribute(Qt::AA_DontCreateNativeWidgetSiblings);
    QApplication app(argc, argv);

    RDT::registerRdtMetaTypes();
    Logger::setLogLevel(LogLevel::Info);

    LOG_INFO("GUI_AppMain", "--- Full GUI Application Starting (Old-Style Init) ---");

    bool LAUNCH_EMULATOR_AUTOMATICALLY = true;
    // --- 0. AUTO-LAUNCH EMULATOR (New Feature) ---
    if (LAUNCH_EMULATOR_AUTOMATICALLY) {
        LOG_INFO("TestMain", "Automatically launching Python emulator in the background...");
        // This command assumes the executable is run from a 'build' directory
        // located at the project root. The '&' makes it run in the background on Linux.
        const char* command = "xterm -T \"Robot Emulator\" -hold -e \"python3  /home/rdt/Desktop/RDT/RobotControl_MVP/motion_interface_hal/robot_utility.py --emulator\" &";
        int result = std::system(command);
        if (result != 0) {
            LOG_ERROR("TestMain", "Failed to launch emulator script. Please run it manually.");
            // We can choose to continue or exit. Let's try to continue.
        }
        LOG_INFO("TestMain", "Waiting 1 second for emulator to initialize...");
    
    } else {
        LOG_INFO("TestMain", "Manual mode: Ensure python/robot_utility.py --emulator is running.");
    }



    // ============================================================================
    // 1. КОНФИГУРАЦИЯ
    // ============================================================================
    InterfaceConfig hw_config;
    hw_config.realtime_type = InterfaceConfig::RealtimeInterfaceType::Udp;
    hw_config.simulation_initial_joints.fromAngleArray({0.0_rad, (-30.0_deg).toRadians(), (30.0_deg).toRadians(), 0.0_rad, 0.0_rad, 0.0_rad});

    ControllerConfig ctrl_config;
    
    // ============================================================================
    // 2. СОЗДАНИЕ КОМПОНЕНТОВ
    // ============================================================================
    LOG_INFO("GUI_AppMain", "Creating components...");
    
    auto state_data = std::make_shared<StateData>();
    KinematicModel kuka_model = KinematicModel::createKR6R900();
    auto robot_controller = std::make_shared<RobotController>(hw_config, ctrl_config, kuka_model, state_data);
    
    auto* adapter = new Adapter_RobotController(robot_controller, state_data, &app);
    
    auto* teach_panel_logic = new Panel_Teach(); 
    QWidget* teach_panel_widget = teach_panel_logic->getWidget();
    auto* robot_view_panel = new Panel_RobotView3D();
    auto* jog_panel = new Panel_JogControl();
    auto* state_display_panel = new Panel_StateDisplay();
    auto* status_bar_panel = new Panel_StatusBar();

    // ============================================================================
    // 3. СБОРКА ГЛАВНОГО ОКНА
    // ============================================================================
    QMainWindow main_window;
    main_window.setWindowTitle("RDT - Industrial Robot Control Panel");
    main_window.resize(1800, 1000);

    auto* central_widget = new QWidget();
    main_window.setCentralWidget(central_widget);
    
    auto* top_main_area_frame = new QFrame(central_widget);
    auto* top_main_grid_layout = new QGridLayout(top_main_area_frame);
    top_main_grid_layout->setContentsMargins(0,0,0,0); 
    top_main_grid_layout->setSpacing(5);
    top_main_grid_layout->addWidget(teach_panel_widget, 0, 0);
    top_main_grid_layout->addWidget(robot_view_panel, 0, 1);
    top_main_grid_layout->addWidget(jog_panel, 0, 2);
    top_main_grid_layout->setColumnStretch(1, 1);

    auto* main_vertical_layout = new QVBoxLayout(central_widget);
    main_vertical_layout->setContentsMargins(5,5,5,5); 
    main_vertical_layout->setSpacing(5);
    main_vertical_layout->addWidget(status_bar_panel);
    main_vertical_layout->addWidget(top_main_area_frame, 1);
    main_vertical_layout->addWidget(state_display_panel);
    
    // ============================================================================
    // 4. ПОДКЛЮЧЕНИЕ ПАНЕЛЕЙ К АДАПТЕРУ
    // ============================================================================
    adapter->connectStateDisplayPanel(state_display_panel);
    adapter->connectRobotView3D(robot_view_panel);
    adapter->connectStatusBarPanel(status_bar_panel);
    adapter->connectTeachPanel(teach_panel_logic);
    adapter->connectJogPanel(jog_panel);
    
    // ============================================================================
    // 5. ИНИЦИАЛИЗАЦИЯ И ЗАПУСК (В СТИЛЕ СТАРОГО ФАЙЛА)
    // ============================================================================
    
    // Сначала показываем окно
    main_window.show();
    LOG_INFO("GUI_AppMain", "Main window shown. Initializing backend...");
    
    // Затем, до app.exec(), вызываем инициализацию, которая внутри загрузит модель
    TrajectoryPoint initial_cmd;
    initial_cmd.command.joint_target = hw_config.simulation_initial_joints;
    
    if (adapter->initializeBackend(initial_cmd)) {
        LOG_INFO("GUI_AppMain", "Backend initialized successfully.");
        
        // *** КЛЮЧЕВОЙ МОМЕНТ: Загружаем модель здесь, как в старом файле ***
        if (robot_view_panel->loadmodel_r900()) {
            LOG_INFO("GUI_AppMain", "Robot 3D model loaded.");
            // И сразу после загрузки отправляем команду на обновление позы
            adapter->currentJointsChanged(initial_cmd.command.joint_target);
        } else {
            LOG_WARN("GUI_AppMain", "Failed to load robot 3D model.");
        }

    } else {
        QMessageBox::critical(&main_window, "Startup Error", "Failed to initialize robot controller backend!");
        return -1;
    }
    
    LOG_INFO("GUI_AppMain", "Starting event loop...");
    return app.exec();
}