cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(MotionInterfaceModule LANGUAGES CXX)

# ============================================================================
# Опция для сборки интерактивного тестера
# ============================================================================
set(BUILD_AXIS_TESTER ON)

# ============================================================================
# Custom dependency search path
# ============================================================================
set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/RDT/libs/")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")

# ============================================================================
# Language & standard
# ============================================================================
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ============================================================================
# Поиск зависимостей
# ============================================================================
# --- Python (для основного приложения) ---
find_package(Python3 COMPONENTS Interpreter)

# --- EtherCAT (для тестера и модуля) ---
find_library(ETHERCAT_LIBRARY ethercat)
if(NOT ETHERCAT_LIBRARY)
    message(FATAL_ERROR "EtherCAT library (libethercat) not found. Please install it (e.g., sudo apt-get install libethercat-dev).")
else()
    message(STATUS "Found EtherCAT library: ${ETHERCAT_LIBRARY}")
endif()

# --- Third-party: tinyxml2 ---
find_package(tinyxml2 CONFIG REQUIRED)

# ============================================================================
# Paths to dependent modules
# ============================================================================
get_filename_component(ROBOT_MVP_ROOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/.. ABSOLUTE)
set(DATA_TYPES_DIR ${ROBOT_MVP_ROOT_DIR}/data_types_rdt)
set(LOGGER_DIR     ${ROBOT_MVP_ROOT_DIR}/logger)

message(STATUS "ROBOT_MVP_ROOT_DIR:      ${ROBOT_MVP_ROOT_DIR}")
message(STATUS "DATA_TYPES_DIR:          ${DATA_TYPES_DIR}")
message(STATUS "LOGGER_DIR:              ${LOGGER_DIR}")

# ============================================================================
# Include directories
# ============================================================================
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/ethercat
    ${DATA_TYPES_DIR}
    ${LOGGER_DIR}
)

# ============================================================================
# Sources
# ============================================================================
# Collect source files, preferring files in the current source dir and falling back to the 'ethercat' subdir
set(MOTION_INTERFACE_MODULE_CPP_SOURCES)

macro(add_optional_src filename)
    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${filename}")
        list(APPEND MOTION_INTERFACE_MODULE_CPP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/${filename}")
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/${filename}")
        list(APPEND MOTION_INTERFACE_MODULE_CPP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/${filename}")
    else()
        message(FATAL_ERROR "Required source file not found: ${filename} (looked in ${CMAKE_CURRENT_SOURCE_DIR} and ${CMAKE_CURRENT_SOURCE_DIR}/ethercat)")
    endif()
endmacro()

add_optional_src(InternalSimulation.cpp)
add_optional_src(MasterHardwareInterface.cpp)
add_optional_src(EthercatInterface.cpp)
add_optional_src(UdpInterface.cpp)
add_optional_src(EthercatAxis.cpp)

# DataTypes.cpp lives in DATA_TYPES_DIR
list(APPEND MOTION_INTERFACE_MODULE_CPP_SOURCES "${DATA_TYPES_DIR}/DataTypes.cpp")

# ============================================================================
# Сборка в зависимости от опции
# ============================================================================

if(BUILD_AXIS_TESTER)
    # --- Сборка интерактивного тестера EtherCAT ---
    message(STATUS "Option BUILD_AXIS_TESTER is ON. Building EthercatAxisTester.")
    
    set(TESTER_MAIN_SRC "${CMAKE_CURRENT_SOURCE_DIR}/ethercat/ethercat_axis_test_main.cpp")
    if(NOT EXISTS "${TESTER_MAIN_SRC}")
        message(FATAL_ERROR "Tester source file not found: ${TESTER_MAIN_SRC}")
    endif()

    add_executable(EthercatAxisTester
        ${TESTER_MAIN_SRC}
        ${MOTION_INTERFACE_MODULE_CPP_SOURCES}
    )

    # Линковка необходимых библиотек для тестера
    target_link_libraries(EthercatAxisTester PRIVATE
        atomic
        ${ETHERCAT_LIBRARY}
        tinyxml2::tinyxml2
    )
    
    # Для sched_setscheduler и mlockall нужна rt библиотека
    target_link_libraries(EthercatAxisTester PRIVATE rt)


else()
    # --- Сборка основного приложения (по умолчанию) ---
    message(STATUS "Option BUILD_AXIS_TESTER is OFF. Building MotionInterfaceTest.")

    set(MAIN_APP_SRC "${CMAKE_CURRENT_SOURCE_DIR}/motion_interface_main.cpp")
    if(NOT EXISTS "${MAIN_APP_SRC}")
        message(FATAL_ERROR "Main source file not found: ${MAIN_APP_SRC}")
    endif()

    add_executable(MotionInterfaceTest
        ${MAIN_APP_SRC}
        ${MOTION_INTERFACE_MODULE_CPP_SOURCES}
    )

    # Передача пути к Python в C++ код
    if(Python3_FOUND)
        target_compile_definitions(MotionInterfaceTest PRIVATE 
            PYTHON_EXECUTABLE="${Python3_EXECUTABLE}"
        )
    else()
        message(WARNING "Python3 interpreter not found. Auto-launching emulator will likely fail.")
    endif()

    # Линковка библиотек для основного приложения
    target_link_libraries(MotionInterfaceTest PRIVATE
        atomic
        ${ETHERCAT_LIBRARY}
        tinyxml2::tinyxml2
    )
    
    # Для sched_setscheduler и mlockall нужна rt библиотека
    target_link_libraries(MotionInterfaceTest PRIVATE rt)

    if(WIN32)
        target_link_libraries(MotionInterfaceTest PRIVATE ws2_32)
    endif()

endif()


# ============================================================================
# Doxygen (остается без изменений)
# ============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
    # ... (ваш код для Doxygen остается здесь без изменений) ...
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