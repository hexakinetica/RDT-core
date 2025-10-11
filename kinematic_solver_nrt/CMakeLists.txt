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
project(kinematic_solver_nrt LANGUAGES CXX)

# ============================================================================
# Language & standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Dependencies
# ============================================================================
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
find_package(orocos_kdl REQUIRED)   # underscore module name

message(STATUS "orocos_kdl_INCLUDE_DIRS = ${orocos_kdl_INCLUDE_DIRS}")
message(STATUS "orocos_kdl_LIBRARIES    = ${orocos_kdl_LIBRARIES}")

# ============================================================================
# Library (module itself)
# ============================================================================
add_library(kinematic_solver_nrt STATIC
    KinematicModel.cpp
    KdlKinematicSolver.cpp
)

target_include_directories(kinematic_solver_nrt
    PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
        ${orocos_kdl_INCLUDE_DIRS}
)

target_link_libraries(kinematic_solver_nrt
    PUBLIC
        Eigen3::Eigen
        ${orocos_kdl_LIBRARIES}
)

# ============================================================================
# Executable (test app for this module)
# ============================================================================
add_executable(kinematic_solver_test
    kinematic_solver_main.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt/DataTypes.cpp
)

target_link_libraries(kinematic_solver_test
    PRIVATE
        kinematic_solver_nrt
)

target_include_directories(kinematic_solver_test
    PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt
        ${CMAKE_CURRENT_SOURCE_DIR}/../logger
)

# ============================================================================
# RPATH (so binary can find liborocos-kdl.so in libs/kdl_install/lib)
# ============================================================================
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(kinematic_solver_test PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)

# ============================================================================
# Warnings (optional)
# ============================================================================
# add_compile_options(-Wall -Wextra -Wpedantic)


# ============================================================================
# Doxygen
# ============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "KinematicsKDL Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_kinematics_kdl")

    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/KinematicSolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/KinematicModel.h"
        "${CMAKE_CURRENT_LIST_DIR}/KinematicModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/kinematic_solver_main.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/Units.h"
        "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.h"
        "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/../logger/Logger.h"
    )

    set(PREDEFINED_GUARDS_KDL
        "KINEMATICSOLVER_H" "KINEMATICMODEL_H" "KDLKINEMATICSOLVER_H"
        "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H"
    )
 
    
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")
    string(REPLACE ";" " " PREDEFINED_GUARDS_KDL_STRING "${PREDEFINED_GUARDS_KDL}")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_kinematics_kdl "
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
        PREDEFINED             = ${PREDEFINED_GUARDS_KDL_STRING}
        QUIET                  = YES
    ")

    add_custom_target(docs_kinematics_kdl
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_kinematics_kdl
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for KinematicsKDL..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_kinematics_kdl' target added.")
else()
    message(WARNING "Doxygen not found. API documentation target 'docs_kinematics_kdl' will not be added.")
endif()
