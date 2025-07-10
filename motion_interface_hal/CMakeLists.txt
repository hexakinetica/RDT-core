# /home/ui/Desktop/Robot_UI/RobotControl_MVP/motion_interface/CMakeLists.txt
cmake_minimum_required(VERSION 3.15)
project(MotionInterfaceTest LANGUAGES CXX) #     MotionInterfaceModule,    MotionInterfaceTest

set(CMAKE_CXX_STANDARD 20) #   20,   17,     C++20.   -  20.
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

#add_compile_options(-Wall -Wextra -Wpedantic -Werror)


# --- Define Paths to Dependent Modules ---
get_filename_component(ROBOT_MVP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(DATA_TYPES_DIR ${ROBOT_MVP_ROOT_DIR}/data_types_rdt)
set(LOGGER_DIR ${ROBOT_MVP_ROOT_DIR}/logger)
get_filename_component(TINYXML2_DIR ${ROBOT_MVP_ROOT_DIR}/../libs/tinyxml2-master ABSOLUTE)

# --- Verify paths (for debugging) ---
message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "ROBOT_MVP_ROOT_DIR: ${ROBOT_MVP_ROOT_DIR}")
message(STATUS "DATA_TYPES_DIR: ${DATA_TYPES_DIR}")
message(STATUS "LOGGER_DIR: ${LOGGER_DIR}")
message(STATUS "TINYXML2_DIR: ${TINYXML2_DIR}")

# --- Include Directories ---
include_directories(
    . # Current directory for local headers
    ${DATA_TYPES_DIR}
    ${LOGGER_DIR}
    ${TINYXML2_DIR}
)

# --- Source Files ---
#    .cpp   
set(MOTION_INTERFACE_MODULE_CPP_SOURCES
    FakeMotionInterface.cpp
    UDPMotionInterface.cpp
    UDPTransport.cpp
    # tinyxml2.cpp -   
    "${TINYXML2_DIR}/tinyxml2.cpp" # ,      
    #    (,    D)
    "${DATA_TYPES_DIR}/DataTypes.cpp" # ,      
)

# ---    ( ) ---
foreach(SOURCE_FILE ${MOTION_INTERFACE_MODULE_CPP_SOURCES})
    if(NOT EXISTS "${SOURCE_FILE}")
        message(WARNING "Source file does not exist: ${SOURCE_FILE}")
    else()
        message(STATUS "Found source file: ${SOURCE_FILE}")
    endif()
endforeach()
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/motion_interface_main.cpp")
    message(FATAL_ERROR "Main source file motion_interface_main.cpp not found in ${CMAKE_CURRENT_SOURCE_DIR}")
endif()


# --- Executable for Testing This Module ---
add_executable(MotionInterfaceTest
    motion_interface_main.cpp #  .cpp    
    ${MOTION_INTERFACE_MODULE_CPP_SOURCES}
)

# --- Link Libraries ---
if(WIN32)
    target_link_libraries(MotionInterfaceTest PRIVATE ws2_32)
endif()


# --- Doxygen ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "Motion Interface Module") #   
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_motion_interface")

    #    .h, .hpp, .cpp   Doxygen
    set(DOXYGEN_INPUT_FILES_LIST
        #   
        "${CMAKE_CURRENT_SOURCE_DIR}/FakeMotionInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/FakeMotionInterface.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/IMotionInterfaces.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/ITransport.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/UDPMotionInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/UDPMotionInterface.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UdpPeer.hpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UDPTransport.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/UDPTransport.cpp"
        #   
        "${DATA_TYPES_DIR}/Units.h"
        "${DATA_TYPES_DIR}/DataTypes.h"
        "${DATA_TYPES_DIR}/DataTypes.cpp"
        "${LOGGER_DIR}/Logger.h"
        # tinyxml2
        "${TINYXML2_DIR}/tinyxml2.h"
        "${TINYXML2_DIR}/tinyxml2.cpp"
    )

    #     Doxygen ( )
    foreach(DOXY_FILE ${DOXYGEN_INPUT_FILES_LIST})
        if(NOT EXISTS "${DOXY_FILE}")
            message(WARNING "Doxygen input file does not exist: ${DOXY_FILE}")
        endif()
    endforeach()

    #    Doxyfile.in
    string(REPLACE ";" "\" \"" DOXYGEN_INPUT_STRING_EXPANDED "\"${DOXYGEN_INPUT_FILES_LIST}\"")

    set(DOXYFILE_IN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Doxygen.in")

    if(EXISTS "${DOXYFILE_IN_PATH}")
        set(DOXYFILE_CONFIGURED "${CMAKE_BINARY_DIR}/Doxyfile_motion_if.configured")
        
        # ,     Doxygen.in  @VAR_NAME@
        # CMAKE_PROJECT_NAME    "MotionInterfaceTest" ( project())
        #         Doxygen,  
        # set(DOXYGEN_PROJECT_TITLE "My Awesome Motion Interface")
        #   @DOXYGEN_PROJECT_TITLE@  Doxygen.in

        configure_file("${DOXYFILE_IN_PATH}" "${DOXYFILE_CONFIGURED}" @ONLY)
        
        add_custom_target(docs_motion_interface COMMAND ${DOXYGEN_EXECUTABLE} "${DOXYFILE_CONFIGURED}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            COMMENT "Generating API documentation with Doxygen for Motion Interface..."
            VERBATIM)
        message(STATUS "Doxygen: 'docs_motion_interface' target added.")
    else()
        message(WARNING "Doxygen: Doxygen.in not found at ${DOXYFILE_IN_PATH}. 'docs_motion_interface' target not added.")
    endif()
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs_motion_interface' will not be added.")
endif()