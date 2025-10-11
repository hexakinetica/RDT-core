cmake_minimum_required(VERSION 3.18)


set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/RDT/libs/")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")

# ============================================================================
# Project
# ============================================================================
project(RobotControlGUI LANGUAGES CXX)


# ============================================================================
# C++ standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Включаем обработку [[nodiscard]] как ошибку
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=unused-result")

# ============================================================================
# Qt Configuration
# ============================================================================
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)

# ============================================================================
# Find Dependencies
# ============================================================================
# Qt6
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL OpenGLWidgets)

# OpenCASCADE
find_package(OpenCASCADE REQUIRED)

# Orocos KDL
find_package(orocos_kdl REQUIRED)

# Eigen3 (header-only)
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

# Threads
find_package(Threads REQUIRED)

# ============================================================================
# Executable Target
# ============================================================================
add_executable(RobotControlApp
    # Главный файл приложения
    RobotControl_main.cpp

    # --- Исходники GUI панелей ---
    Panel_StateDisplay.cpp
    Panel_StatusBar.cpp
    Panel_JogControl.cpp
    Panel_Teach.cpp
    Panel_RobotView3D.cpp
    #draw_primitives.cpp # Вспомогательный класс для 3D

    # --- Исходник Адаптера ---
    Adapter_RobotController.cpp

    # --- Исходники всех необходимых модулей из соседних директорий ---
    
    # Контроллер
    ../robot_controller_nrt/RobotController.cpp
    
    # Менеджер движения
    ../motion_manager_rt/MotionManager.cpp
    
    # Интерфейсы (самое важное добавление)
    ../motion_interface_hal/MasterHardwareInterface.cpp
    ../motion_interface_hal/InternalSimulation.cpp
    ../motion_interface_hal/EthercatInterface.cpp
    ../motion_interface_hal/UdpInterface.cpp
    # Планировщик
    ../trajectory_planner_nrt/TrajectoryPlanner.cpp
    
    # Интерполятор
    ../interpolator_nrt/TrajectoryInterpolator.cpp
    ../interpolator_nrt/MotionProfile.cpp
    
    # Кинематика
    ../kinematic_solver_nrt/KinematicModel.cpp
    ../kinematic_solver_nrt/KdlKinematicSolver.cpp
    
    # Типы данных
    ../data_types_rdt/DataTypes.cpp



)

# ============================================================================
# Include Directories
# ============================================================================
target_include_directories(RobotControlApp PUBLIC
    # Текущая директория для заголовочных файлов панелей
    .
    
    # Пути ко всем модулям проекта
    ../data_types_rdt
    ../logger
    ../trajectory_queue_lf
    ../motion_interface_hal
    ../kinematic_solver_nrt
    ../interpolator_nrt
    ../trajectory_planner_nrt
    ../motion_manager_rt
    ../robot_controller_nrt
    ../state_data_nrt
    ../spatial_math_
    
    # Пути к внешним зависимостям
    ${OpenCASCADE_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

# ============================================================================
# Link Libraries
# ============================================================================
target_link_libraries(RobotControlApp PRIVATE
    # Qt
    Qt6::Core
    Qt6::Gui
    Qt6::Widgets
    Qt6::OpenGL
    Qt6::OpenGLWidgets
    # OpenCASCADE
    ${OpenCASCADE_LIBRARIES}

    # KDL
    ${orocos_kdl_LIBRARIES}
    
    # Другие
    Eigen3::Eigen
    Threads::Threads
    atomic
)

# ============================================================================
# RPATH (для поиска .so файлов во время выполнения)
# (Предполагается, что библиотеки лежат в общей папке libs/lib)
# ============================================================================
set(LIBS_RPATH "${CMAKE_SOURCE_DIR}/../libs/lib")
if(EXISTS ${LIBS_RPATH})
    set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set_target_properties(RobotControlApp PROPERTIES
        BUILD_RPATH "${LIBS_RPATH}"
        INSTALL_RPATH "${LIBS_RPATH}"
    )
endif()