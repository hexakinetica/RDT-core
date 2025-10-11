cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Custom dependency search path (копируем из вашего примера)
# ============================================================================
set(LOCAL_DEPENDENCY_PREFIX "/home/rdt/Desktop/RDT/libs/")
list(APPEND CMAKE_PREFIX_PATH "${LOCAL_DEPENDENCY_PREFIX}")
message(STATUS "Adding custom search path for dependencies: ${LOCAL_DEPENDENCY_PREFIX}")

# ============================================================================
# Project
# ============================================================================
project(motion_manager_test LANGUAGES CXX)

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
# Находим KDL, так как он нужен для KinematicModel, который мы используем
find_package(orocos_kdl REQUIRED)
# Находим потоки
find_package(Threads REQUIRED)

# ============================================================================
# Executable (наш "ебейший тест")
# ============================================================================
add_executable(motion_manager_test
    motion_manager_main.cpp
    MotionManager.cpp
    
    # --- КЛЮЧЕВЫЕ ИЗМЕНЕНИЯ: Добавляем все необходимые .cpp файлы ---
    # Из модуля интерфейсов
    ../motion_interface_hal/MasterHardwareInterface.cpp
    ../motion_interface_hal/InternalSimulation.cpp
    ../motion_interface_hal/EthercatInterface.cpp
    
    # Из модуля кинематики
    ../kinematic_solver_nrt/KinematicModel.cpp
    
    # Из модуля типов данных
    ../data_types_rdt/DataTypes.cpp
)

# ============================================================================
# Include Directories
# ============================================================================
target_include_directories(motion_manager_test PUBLIC
    . # Текущая директория (motion_manager_rt)
    
    # Пути к другим модулям нашего проекта
    ../motion_interface_hal
    ../kinematic_solver_nrt
    ../data_types_rdt
    ../logger
    ../trajectory_queue_lf
    
    # Путь к заголовкам KDL
    ${orocos_kdl_INCLUDE_DIRS}
)

# ============================================================================
# Link Libraries
# ============================================================================
target_link_libraries(motion_manager_test PRIVATE
    # Линкуем с библиотекой KDL, так как KinematicModel.cpp ее использует
    ${orocos_kdl_LIBRARIES}
    
    # Линкуем с библиотекой потоков
    Threads::Threads
)
target_link_libraries(motion_manager_test PRIVATE atomic)
# ============================================================================
# RPATH (копируем из вашего примера для консистентности)
# ============================================================================
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
set_target_properties(motion_manager_test PROPERTIES
    INSTALL_RPATH "\$ORIGIN/../../libs/kdl_install/lib"
    BUILD_RPATH   "${CMAKE_CURRENT_LIST_DIR}/../../libs/kdl_install/lib"
)

# ============================================================================
# Doxygen
# ============================================================================
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "MotionManager Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_motion_manager")

    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/MotionManager.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/MotionManager.cpp"
        "${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt/Units.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt/DataTypes.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/IMotionInterfaces.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/ITransport.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/FakeMotionInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/UDPTransport.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/UDPMotionInterface.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_queue_lf/TrajectoryQueue.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/../logger/Logger.h"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    set(PREDEFINED_GUARDS
        "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H"
        "TRAJECTORYQUEUE_H" "IMOTION_INTERFACES_H" "ITRANSPORT_H"
        "FAKEMOTIONINTERFACE_H" "MOTION_MANAGER_H"
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_STRING "${PREDEFINED_GUARDS}")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_motion_manager "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        PROJECT_NUMBER         = \"${PROJECT_VERSION}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        RECURSIVE              = NO
        EXTRACT_ALL            = YES
        EXTRACT_PRIVATE        = NO
        GENERATE_HTML          = YES
        HTML_OUTPUT            = html
        HAVE_DOT               = YES
        UML_LOOK               = YES
        CLASS_DIAGRAMS         = YES
        CALL_GRAPH             = YES
        CALLER_GRAPH           = YES
        NAMESPACE_DIAGRAMS     = YES
        WARN_IF_UNDOCUMENTED   = YES
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        PREDEFINED             = ${PREDEFINED_GUARDS_STRING}
        QUIET                  = YES
    ")

    add_custom_target(docs_motion_manager
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_motion_manager
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for MotionManager..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_motion_manager' target added.")
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs_motion_manager' will not be added.")
endif()
