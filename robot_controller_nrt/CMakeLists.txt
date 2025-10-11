cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Custom dependency search path (стиль из вашего примера)
# ============================================================================
set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/RDT/libs/")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")

# ============================================================================
# Project
# ============================================================================
project(RobotController_Test LANGUAGES CXX)

# ============================================================================
# Language & standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=unused-result")

# ============================================================================
# Dependencies
# ============================================================================
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "Found orocos_kdl includes: ${orocos_kdl_INCLUDE_DIRS}")
message(STATUS "Found orocos_kdl libraries: ${orocos_kdl_LIBRARIES}")

# ============================================================================
# Executable (Наш тест для RobotController)
# ============================================================================
add_executable(robot_controller_test_app
    robot_controller_main.cpp
    RobotController.cpp

    # --- ЯВНО ДОБАВЛЯЕМ ВСЕ НЕОБХОДИМЫЕ .cpp ФАЙЛЫ ИЗ ДРУГИХ МОДУЛЕЙ ---
    ../data_types_rdt/DataTypes.cpp
    
    ../motion_interface_hal/MasterHardwareInterface.cpp
    ../motion_interface_hal/InternalSimulation.cpp
    ../motion_interface_hal/EthercatInterface.cpp
    
    ../motion_manager_rt/MotionManager.cpp
    
    ../trajectory_planner_nrt/TrajectoryPlanner.cpp
    
    ../interpolator_nrt/TrajectoryInterpolator.cpp
    ../interpolator_nrt/MotionProfile.cpp

    ../kinematic_solver_nrt/KinematicModel.cpp
    ../kinematic_solver_nrt/KdlKinematicSolver.cpp
)

# ============================================================================
# Include Directories
# ============================================================================
# ============================================================================
# Includes
# ============================================================================
target_include_directories(robot_controller_test_app PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                      # RobotController.h
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"   # DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"           # Logger.h
    "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_"    # FrameTransformer.h
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf"  # TrajectoryQueue.h
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt"     # TrajectoryInterpolator.h, MotionProfile.h
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt" # KinematicSolver.h, KinematicModel.h, KdlKinematicSolver.h
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal" # IMotionInterfaces.h, FakeMotionInterface.h, ITransport.h, etc.
    "${CMAKE_CURRENT_LIST_DIR}/../state_data_nrt"       # StateData.h
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_planner_nrt" # TrajectoryPlanner.h
    "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt"      # MotionManager.h

    ${EIGEN3_INCLUDE_DIR}
    ${orocos_kdl_INCLUDE_DIRS}
)

# ============================================================================
# Link Libraries
# ============================================================================
target_link_libraries(robot_controller_test_app PRIVATE
    # Внешние библиотеки
    ${orocos_kdl_LIBRARIES}
    Threads::Threads
    atomic
)

# ============================================================================
# RPATH (стиль из вашего примера)
# ============================================================================
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(robot_controller_test_app PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)