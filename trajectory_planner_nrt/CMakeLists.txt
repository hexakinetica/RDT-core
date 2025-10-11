cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Custom dependency search path (visible & explicit)
# ============================================================================
set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/RDT/libs/") #
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")

# ============================================================================
# Project
# ============================================================================
project(TrajectoryPlannerTest LANGUAGES CXX)

# ============================================================================
# Language & standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Warnings (keep original style)
# ============================================================================
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ============================================================================
# Dependencies
#   - Eigen3 header-only (3.4 per your env)
#   - orocos_kdl via Module name with underscore (exports *_INCLUDE_DIRS/LIBRARIES)
# ============================================================================
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)

message(STATUS "orocos_kdl_INCLUDE_DIRS = ${orocos_kdl_INCLUDE_DIRS}")
message(STATUS "orocos_kdl_LIBRARIES    = ${orocos_kdl_LIBRARIES}")

# ============================================================================
# Executable
# ============================================================================
add_executable(TrajectoryPlannerTestApp
    trajectory_planner_main.cpp
    TrajectoryPlanner.cpp
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/TrajectoryInterpolator.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/MotionProfile.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KinematicModel.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KdlKinematicSolver.cpp"
)

# ============================================================================
# Includes
# ============================================================================
target_include_directories(TrajectoryPlannerTestApp PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                         # TrajectoryPlanner.h
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       # DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"               # Logger.h
    "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_"        # FrameTransformer.h
    "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt"     # TrajectoryInterpolator.h, MotionProfile.h
    "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt" # KinematicSolver.h, KinematicModel.h, KdlKinematicSolver.h
    ${orocos_kdl_INCLUDE_DIRS}
)

# ============================================================================
# Link libraries
# ============================================================================
target_link_libraries(TrajectoryPlannerTestApp PRIVATE
    Eigen3::Eigen
    ${orocos_kdl_LIBRARIES}
)

# ============================================================================
# RPATH (keep KDL runtime path like your original)
# ============================================================================
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(TrajectoryPlannerTestApp PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)

# ============================================================================
# Doxygen
# ============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "TrajectoryPlanner Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_traj_planner")

    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryPlanner.h"
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryPlanner.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/trajectory_planner_main.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/Units.h"
        "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.h"
        "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/TrajectoryInterpolator.h"
        "${CMAKE_CURRENT_LIST_DIR}/../interpolator_nrt/MotionProfile.h"
        "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KinematicModel.h"
        "${CMAKE_CURRENT_LIST_DIR}/../kinematic_solver_nrt/KdlKinematicSolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/../logger/Logger.h"
        "${CMAKE_CURRENT_LIST_DIR}/../spatial_math_/FrameTransformer.h"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    set(PREDEFINED_GUARDS_TP
        "TRAJECTORYPLANNER_H"
        "TRAJECTORYINTERPOLATOR_H" "MOTIONPROFILE_H"
        "KINEMATICSOLVER_H" "KINEMATICMODEL_H" "KDLKINEMATICSOLVER_H"
        "FRAMETRANSFORMER_H" "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H"
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_TP_STRING "${PREDEFINED_GUARDS_TP}")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_traj_planner "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        RECURSIVE              = NO
        EXTRACT_ALL            = YES
        GENERATE_HTML          = YES
        HTML_OUTPUT            = html
        HAVE_DOT               = YES
        CLASS_DIAGRAMS         = YES
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        PREDEFINED             = ${PREDEFINED_GUARDS_TP_STRING}
        QUIET                  = YES
        WARN_IF_UNDOCUMENTED   = YES
    ")

    add_custom_target(docs_traj_planner
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_traj_planner
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API for TrajectoryPlanner..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_traj_planner' target added.")
else()
    message(WARNING "Doxygen not found. 'docs_traj_planner' target not added.")
endif()
