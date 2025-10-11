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
project(MotionInterfaceTest LANGUAGES CXX) # MotionInterfaceModule, MotionInterfaceTest

# ============================================================================
# Language & standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20) # keep C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# NOTE: keep your original flags pattern; enable if you want strict mode
# add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# ============================================================================
# Paths to dependent modules (keep original layout & messages)
# ============================================================================
get_filename_component(ROBOT_MVP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(DATA_TYPES_DIR ${ROBOT_MVP_ROOT_DIR}/data_types_rdt)
set(LOGGER_DIR     ${ROBOT_MVP_ROOT_DIR}/logger)

message(STATUS "CMAKE_CURRENT_SOURCE_DIR: ${CMAKE_CURRENT_SOURCE_DIR}")
message(STATUS "ROBOT_MVP_ROOT_DIR:      ${ROBOT_MVP_ROOT_DIR}")
message(STATUS "DATA_TYPES_DIR:          ${DATA_TYPES_DIR}")
message(STATUS "LOGGER_DIR:              ${LOGGER_DIR}")

# ============================================================================
# Third-party: tinyxml2 via find_package (installed by your script)
# ============================================================================
find_package(tinyxml2 CONFIG REQUIRED)

# ============================================================================
# Include directories (keep original style)
# ============================================================================
include_directories(                       # local headers
    ${DATA_TYPES_DIR}
    ${LOGGER_DIR}
)

# ============================================================================
# Sources (keep both Fake and UDP like original; do NOT pull tinyxml2.cpp)
# ============================================================================
set(MOTION_INTERFACE_MODULE_CPP_SOURCES
    InternalSimulation.cpp
    MasterHardwareInterface.cpp
    InternalSimulation.cpp
    EthercatInterface.cpp
    UdpInterface.cpp
    "${DATA_TYPES_DIR}/DataTypes.cpp"   # preserve original inclusion
)


if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/motion_interface_main.cpp")
    message(FATAL_ERROR "Main source file motion_interface_main.cpp not found in ${CMAKE_CURRENT_SOURCE_DIR}")
endif()

# ============================================================================
# Executable
# ============================================================================
add_executable(MotionInterfaceTest
    motion_interface_main.cpp
    ${MOTION_INTERFACE_MODULE_CPP_SOURCES}
)
# --- MODIFIED: Pass Python path to C++ code ---
# We pass the found path as a preprocessor definition.
# The quotes are important to handle paths with spaces.
target_compile_definitions(MotionInterfaceTest PRIVATE 
    PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
)
# Link system libs on Windows if you ever flip toolchain; harmless on Linux
if(WIN32)
    target_link_libraries(MotionInterfaceTest PRIVATE ws2_32)
endif()

# Link tinyxml2 properly (no direct .cpp)
target_link_libraries(MotionInterfaceTest PRIVATE tinyxml2::tinyxml2)

target_link_libraries(MotionInterfaceTest PRIVATE atomic)

# ============================================================================
# Doxygen (preserve your workflow; use Doxygen.in if present)
# ============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "Motion Interface Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_motion_interface")

    set(DOXYGEN_INPUT_FILES_LIST
        "${CMAKE_CURRENT_SOURCE_DIR}/MasterHardwareInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/MasterHardwareInterface.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/InternalSimulation.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/InternalSimulation.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/EthercatInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/EthercatInterface.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/UdpPeer.hpp"
  
        "${DATA_TYPES_DIR}/Units.h"
        "${DATA_TYPES_DIR}/DataTypes.h"
        "${DATA_TYPES_DIR}/DataTypes.cpp"
        "${LOGGER_DIR}/Logger.h"
    )

    foreach(DOXY_FILE ${DOXYGEN_INPUT_FILES_LIST})
        if(NOT EXISTS "${DOXY_FILE}")
            message(WARNING "Doxygen input file does not exist: ${DOXY_FILE}")
        endif()
    endforeach()

    set(DOXYFILE_IN_PATH "${CMAKE_CURRENT_SOURCE_DIR}/Doxygen.in")
    if(EXISTS "${DOXYFILE_IN_PATH}")
        set(DOXYFILE_CONFIGURED "${CMAKE_BINARY_DIR}/Doxyfile_motion_if.configured")
        # Example: you can define DOXYGEN_PROJECT_TITLE in cache or here
        set(DOXYGEN_PROJECT_TITLE "Motion Interface Module")
        configure_file("${DOXYFILE_IN_PATH}" "${DOXYFILE_CONFIGURED}" @ONLY)

        add_custom_target(docs_motion_interface
            COMMAND ${DOXYGEN_EXECUTABLE} "${DOXYFILE_CONFIGURED}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            COMMENT "Generating API documentation with Doxygen for Motion Interface..."
            VERBATIM
        )
        message(STATUS "Doxygen: 'docs_motion_interface' target added.")
    else()
        # Minimal inline Doxygen config as fallback
        set(_doxy_inline "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile.motion_if")
        string(REPLACE ";" " " _doxy_inputs "${DOXYGEN_INPUT_FILES_LIST}")
        file(WRITE "${_doxy_inline}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
INPUT                  = ${_doxy_inputs}
RECURSIVE              = NO
EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = YES
GENERATE_HTML          = YES
HTML_OUTPUT            = html
HAVE_DOT               = YES
UML_LOOK               = YES
CLASS_DIAGRAMS         = YES
NAMESPACE_DIAGRAMS     = YES
WARN_IF_UNDOCUMENTED   = YES
ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
")
        add_custom_target(docs_motion_interface
            COMMAND ${DOXYGEN_EXECUTABLE} "${_doxy_inline}"
            WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
            COMMENT "Generating API documentation with Doxygen for Motion Interface (inline config)..."
            VERBATIM
        )
        message(STATUS "Doxygen: 'docs_motion_interface' target added (inline config).")
    endif()
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs_motion_interface' will not be added.")
endif()
