# CMakeLists.txt   Panel_StateDisplay
message(STATUS "   VSCode:  launch.json  LD_LIBRARY_PATH=/home/ui/Desktop/Robot_UI/libs/occt-install/lib  QT_QPA_PLATFORM=xcb")

cmake_minimum_required(VERSION 3.20)
project(StateDisplayPanelTest LANGUAGES CXX)

# ---   Qt6 ---
set(Qt6_DIR_HINT "/usr/lib/x86_64-linux-gnu/cmake/Qt6") # <--     
set(CMAKE_PREFIX_PATH "${Qt6_DIR_HINT}" CACHE STRING "Path to Qt6" FORCE)

# ---  C++ ---
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---  Qt ---
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC_PATH_PREFIX ON)

# --- Qt- ---
find_package(Qt6 REQUIRED COMPONENTS Core Gui Widgets OpenGL  OpenGLWidgets)

set(APP_NAME "cs_test")

# ---    ---
# Panel_StateDisplay.h    #include  .cpp 
add_executable(${APP_NAME}
     RobotControl_main.cpp    

    #panel_statedisplay_main.cpp  #panel_statedisplay
    Panel_StateDisplay.h         #panel_statedisplay
    Panel_StateDisplay.cpp       #panel_statedisplay  

    #panel_statusbar_main.cpp      #panel_statusbar
    Panel_StatusBar.h             #panel_statusbar
    Panel_StatusBar.cpp           #panel_statusbar

    #panel_jogcontrol_main.cpp     #panel_jogcontrol
    Panel_JogControl.h            #panel_jogcontrol
    Panel_JogControl.cpp          #panel_jogcontrol

    #panel_teach_main.cpp           #panel_teach
    Panel_Teach.h                  #panel_teach
    Panel_Teach.cpp                #panel_teach

    #panel_robotview3d_main.cpp      #panel_robotview3d
    Panel_RobotView3D.h             #panel_robotview3d
    Panel_RobotView3D.cpp           #panel_robotview3d

    #draw_primitives.h             #panel_robotview3d
    #draw_primitives.cpp           #panel_robotview3d


    Adapter_RobotController.h
    Adapter_RobotController.cpp
    #Mock_Adapter.h
    #Mock_Adapter.cpp

    #  .cpp   
    # ,        CMakeLists.txt
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp" 
    "${CMAKE_CURRENT_LIST_DIR}/../state_data_nrt/StateData.h" 
    "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_/FrameTransformer.h"

    "${CMAKE_CURRENT_LIST_DIR}/../robot_controller_nrt/RobotController.cpp"

    #  .cpp   
    "${CMAKE_CURRENT_LIST_DIR}/../motion_manager_rt/MotionManager.cpp"           #     
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_planner_nrt/TrajectoryPlanner.cpp" #     
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/TrajectoryInterpolator.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/MotionProfile.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KinematicModel.cpp"            #     
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KdlKinematicSolver.cpp"        #     
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/FakeMotionInterface.cpp" #     
    # ${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/UDPTransport.cpp #   UDP  
    # ${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal/UDPMotionInterface.cpp #   UDP  
)

# ---      ---
target_include_directories(${APP_NAME} PUBLIC
    # ,   Panel_StateDisplay.h ( main, cpp)
    "${CMAKE_CURRENT_LIST_DIR}/../robot_controller_nrt" #  RobotController.h

    "${CMAKE_CURRENT_LIST_DIR}"                         
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

    # "${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0" 
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/occt-install/include/opencascade" # OpenCASCADE
    "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/kdl_install/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/eigen-3.4.0"

)


message(STATUS "Qt6_INCLUDE_DIRS: ${Qt6_INCLUDE_DIRS}")
message(STATUS "Qt6_LIBRARIES: ${Qt6_LIBRARIES}")


add_custom_command(TARGET ${APP_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo "#!/bin/bash" > run.sh
    COMMAND ${CMAKE_COMMAND} -E echo "export LD_LIBRARY_PATH=${OCC_LIB_PATH}:\$LD_LIBRARY_PATH" >> run.sh
    COMMAND ${CMAKE_COMMAND} -E echo "export QT_QPA_PLATFORM=xcb" >> run.sh
    COMMAND ${CMAKE_COMMAND} -E echo "./${APP_NAME} \"\$@\"" >> run.sh
    COMMAND chmod +x run.sh
)


set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/kdl_install/lib;${CMAKE_CURRENT_SOURCE_DIR}/../../libs/occt-install/lib"
)

set(LIBS_DIR                 "${CMAKE_CURRENT_LIST_DIR}/../../libs")
# ---    ---
target_link_libraries(${APP_NAME} PRIVATE
    # Qt
    Qt::Core
    Qt::Gui
    Qt::Widgets
    Qt::OpenGL
    Qt::OpenGLWidgets #  QOpenGLWidget

    # KDL ( )
    "${LIBS_DIR}/kdl_install/lib/liborocos-kdl.so"
    "${LIBS_DIR}/occt-install/lib/libTKernel.so"
    "${LIBS_DIR}/occt-install/lib/libTKMath.so"
    "${LIBS_DIR}/occt-install/lib/libTKG2d.so"
    "${LIBS_DIR}/occt-install/lib/libTKG3d.so"
    "${LIBS_DIR}/occt-install/lib/libTKGeomBase.so"
    "${LIBS_DIR}/occt-install/lib/libTKGeomAlgo.so"
    "${LIBS_DIR}/occt-install/lib/libTKTopAlgo.so"
    "${LIBS_DIR}/occt-install/lib/libTKBRep.so"
    "${LIBS_DIR}/occt-install/lib/libTKPrim.so" #  BRepPrimAPI_Make...
    "${LIBS_DIR}/occt-install/lib/libTKService.so"
    "${LIBS_DIR}/occt-install/lib/libTKDESTEP.so"
    "${LIBS_DIR}/occt-install/lib/libTKXSBase.so"
    "${LIBS_DIR}/occt-install/lib/libTKCAF.so"   #  STEPCAFControl_Controller
    "${LIBS_DIR}/occt-install/lib/libTKXCAF.so"  #  XCAFDoc_DocumentTool
    "${LIBS_DIR}/occt-install/lib/libTKV3d.so"   #  V3d_Viewer, View
    "${LIBS_DIR}/occt-install/lib/libTKOpenGl.so" #  OpenGl_GraphicDriver
    # TKBool, TKDESTEP, TKShHealing, TKLCAF
   "${LIBS_DIR}/occt-install/lib/libTKLCAF.so"

    # Pthreads ( std::jthread, std::thread, std::mutex)
    Threads::Threads 
)


# --- RPATH for Runtime Linking ---
set_target_properties(${APP_NAME} PROPERTIES
    INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/kdl_install/lib;${OCC_LIB_PATH}"
)





# --- Doxygen ---
# ...