# CMakeLists.txt   robot_controller

cmake_minimum_required(VERSION 3.15)
project(RobotControllerTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---   ---
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---     ---
add_executable(RobotControllerTestApp
    robot_controller_main.cpp
    RobotController.cpp

    #  .cpp   
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/MotionManager.cpp"           #     
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_planner_nrt/TrajectoryPlanner.cpp" #     
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/TrajectoryInterpolator.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/MotionProfile.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KinematicModel.cpp"            #     
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KdlKinematicSolver.cpp"        #     
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/FakeMotionInterface.cpp" #     
    # ${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/UDPTransport.cpp #   UDP  
    # ${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/UDPMotionInterface.cpp #   UDP  

    # Header-only : Logger.h, Units.h, FrameTransformer.h, StateData.h,
    # IMotionInterfaces.h, ITransport.h, UdpPeer.hpp, TrajectoryQueue.h
)

# ---      ---
target_include_directories(RobotControllerTestApp PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                         #  RobotController.h
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       # DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"                # Logger.h
    "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_"          # FrameTransformer.h
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf"   # TrajectoryQueue.h
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt"      # TrajectoryInterpolator.h, MotionProfile.h
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt"        # KinematicSolver.h, KinematicModel.h, KdlKinematicSolver.h
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal"  # IMotionInterfaces.h, FakeMotionInterface.h, ITransport.h, etc
    "${CMAKE_CURRENT_LIST_DIR}/../state_data_nrt"        # StateData.h (  )
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_planner_nrt"        # TrajectoryPlanner.h (  )
    "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt"        # MotionManager.h (  )

    "${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0"   # Eigen3
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/include" # KDL includes
)

# ---    ---
target_link_libraries(RobotControllerTestApp PRIVATE
    # KDL
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib/liborocos-kdl.so"
    # Pthreads ( std::jthread, std::thread, std::mutex)
    #Threads::Threads # CMake     pthreads
)
if(WIN32) #   ,  UDP 
    target_link_libraries(RobotControllerTestApp PRIVATE ws2_32)
endif()


# --- RPATH  KDL ---
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(RobotControllerTestApp PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)

# --- Doxygen ---
# ( ,   .h  .cpp      )
# ...