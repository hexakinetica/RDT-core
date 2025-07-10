# CMakeLists.txt  kinematics_kdl

cmake_minimum_required(VERSION 3.15)
project(KinematicsKDLTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---   ---
# if(MSVC) ... else ... endif()  - 
# add_compile_options(-Wall -Wextra -Wpedantic)

# ---      ---
add_executable(KinematicsKDLTestApp
    #    (kinematics_kdl)
    kinematic_solver_main.cpp
    KinematicModel.cpp
    KdlKinematicSolver.cpp

    #  .cpp   
    #   DataTypes.cpp  CMAKE_CURRENT_LIST_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp" # 
    # Logger.h - header-only
)

# ---      ---
# CMAKE_CURRENT_LIST_DIR -  /path/to/RobotControl_MVP/kinematics_kdl/
target_include_directories(KinematicsKDLTestApp PUBLIC
    #    
    "${CMAKE_CURRENT_LIST_DIR}"

    #     ()
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt" #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"         #  Logger.h
    # TrajectoryQueue     ,     :
    # "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf" 

    #   Eigen3  KDL   libs
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0"
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/include"
)

# ---    KDL ---
target_link_libraries(KinematicsKDLTestApp PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib/liborocos-kdl.so"
)

# ---  RPATH ---
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(KinematicsKDLTestApp PROPERTIES
    INSTALL_RPATH "$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)

# --- Doxygen    ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "KinematicsKDL Module (Direct Paths)")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_kinematics_kdl_direct")

    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/KinematicSolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/KinematicModel.h"
        "${CMAKE_CURRENT_LIST_DIR}/KinematicModel.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver.h"
        "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/main.cpp"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    set(PREDEFINED_GUARDS_KDL
        "KINEMATICSOLVER_H" "KINEMATICMODEL_H" "KDLKINEMATICSOLVER_H"
        "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H"
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_KDL_STRING "${PREDEFINED_GUARDS_KDL}")

    # INCLUDE_PATH  Doxygen ()
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_kinematics_kdl_direct "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../logger\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/include\"
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
    add_custom_target(docs_kinematics_kdl COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_kinematics_kdl_direct
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for KinematicsKDL (Direct Paths)..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_kinematics_kdl' target added (direct paths).")
else()
    message(WARNING "Doxygen not found. API documentation target 'docs_kinematics_kdl' will not be added.")
endif()