# CMakeLists.txt   trajectory_planner

cmake_minimum_required(VERSION 3.15)
project(TrajectoryPlannerTest LANGUAGES CXX)

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
add_executable(TrajectoryPlannerTestApp
    trajectory_planner_main.cpp
    TrajectoryPlanner.cpp
    # TrajectoryInterpolator.h  MotionProfile.h - header-only   .cpp  ,
    #       header-only.
    #    ,    .h  .cpp.
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/TrajectoryInterpolator.cpp" #    
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/MotionProfile.cpp"       #    

    #  .cpp   
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KinematicModel.cpp" #  KinematicModel  .cpp
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KdlKinematicSolver.cpp" #  KdlKinematicSolver  .cpp
    # Logger.h, Units.h, FrameTransformer.h - header-only
    # KdlKinematicSolver_mock.h - header-only
)

# ---      ---
target_include_directories(TrajectoryPlannerTestApp PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                         #  TrajectoryPlanner.h
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"                #  Logger.h
    "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_"          #  FrameTransformer.h
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt"      #  TrajectoryInterpolator.h, MotionProfile.h
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt"        #  KinematicSolver.h, KinematicModel.h, KdlKinematicSolver_mock.h
    
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0"   #   Eigen3
    #   KDL ( KdlKinematicSolver  mock,  )
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/include"
)

# ---    ---
#  KdlKinematicSolver ,   mock:
target_link_libraries(TrajectoryPlannerTestApp PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib/liborocos-kdl.so"
)
# RPATH  KDL
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(TrajectoryPlannerTestApp PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)


# --- Doxygen ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "TrajectoryPlanner Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_traj_planner")
    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryPlanner.h"
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryPlanner.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trajectory_planner_main.cpp"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")
    set(PREDEFINED_GUARDS_TP
        "TRAJECTORYPLANNER_H"
        "TRAJECTORYINTERPOLATOR_H" "MOTIONPROFILE_H"
        "KINEMATICSOLVER_H" "KINEMATICMODEL_H" "KDLKINEMATICSOLVER_MOCK_H"
        "FRAMETRANSFORMER_H" "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H"
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_TP_STRING "${PREDEFINED_GUARDS_TP}")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_traj_planner "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../logger\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../spatial_math\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../kinematics_kdl\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/include\"
        RECURSIVE              = NO EXTRACT_ALL = YES GENERATE_HTML = YES HTML_OUTPUT = html
        HAVE_DOT               = YES CLASS_DIAGRAMS = YES ENABLE_PREPROCESSING = YES
        MACRO_EXPANSION        = YES PREDEFINED = ${PREDEFINED_GUARDS_TP_STRING}
        QUIET                  = YES WARN_IF_UNDOCUMENTED = YES
    ")
    add_custom_target(docs_traj_planner COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_traj_planner
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API for TrajectoryPlanner..." VERBATIM)
    message(STATUS "Doxygen: 'docs_traj_planner' target added.")
else()
    message(WARNING "Doxygen not found. 'docs_traj_planner' target not added.")
endif()