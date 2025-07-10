cmake_minimum_required(VERSION 3.15)
project(MotionManagerRT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---       () ---
if(MSVC)
    add_compile_options(/W4 /WX)
else() # GCC/Clang
    add_compile_options(-Wall -Wextra -Wpedantic -Werror)
endif()

# ---     MotionManager ---
add_executable(MotionManagerTestApp
    motion_manager_main.cpp
    MotionManager.cpp
    #  tinyxml2      (     motion_interface_hal)
    # ${CMAKE_CURRENT_SOURCE_DIR}/../path/to/tinyxml2.cpp 
)

# ---      ---
# CMAKE_CURRENT_SOURCE_DIR   /path/to/RobotControl_MVP/motion_manager_rt/
target_include_directories(MotionManagerTestApp PUBLIC
    #    
    "${CMAKE_CURRENT_SOURCE_DIR}" 

    #      (     )
    # ,   CMakeLists.txt   motion_manager_rt,
    #   CMakeLists.txt      RobotControl_MVP (  ,  RobotControl_MVP - ).
    #  RobotControl_MVP -   ,  ${CMAKE_SOURCE_DIR}    .
    #  ${CMAKE_CURRENT_SOURCE_DIR}/..     RobotControl_MVP
    "${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt"       #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_SOURCE_DIR}/../logger"                #  Logger.h
    "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal"  #  IMotionInterfaces.h, FakeMotionInterface.h, etc.
    "${CMAKE_CURRENT_SOURCE_DIR}/../trajectory_queue_lf"   #  TrajectoryQueue.h
    
    #  tinyxml2    include
    # "${CMAKE_CURRENT_SOURCE_DIR}/../path/to/tinyxml2_include" 
)

# ---       ---
# ,   CMakeLists.txt    (targets)
#     ,   .cpp   , 
#  target_link_libraries  ,  .cpp     
#  MotionManagerTestApp    / ,   .

#   ,    .cpp   
#  MotionManagerTestApp,      , 
#  CMakeLists.txt    CMakeLists.txt  .

# ,   data_types_rdt   DataTypesLibRDT:
# target_link_libraries(MotionManagerTestApp PRIVATE DataTypesLibRDT)
# ,   motion_interface_hal   MotionInterfaceHAL:
# target_link_libraries(MotionManagerTestApp PRIVATE MotionInterfaceHAL)

#    .cpp        :
target_sources(MotionManagerTestApp PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/../data_types_rdt/DataTypes.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/FakeMotionInterface.cpp"
    #  UDPMotionInterface   main.cpp  MotionManager,   .cpp:
    # "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/UDPMotionInterface.cpp"
    # "${CMAKE_CURRENT_SOURCE_DIR}/../motion_interface_hal/UDPTransport.cpp"
    #  tinyxml2  :
    # "${CMAKE_CURRENT_SOURCE_DIR}/../path/to/tinyxml2.cpp"
)


# ---    ( UDPMotionInterface   ) ---
if(WIN32)
    target_link_libraries(MotionManagerTestApp PRIVATE ws2_32)
endif()
#  Linux/macOS       ,    pthreads.

# --- Doxygen ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "MotionManager Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_motion_manager") #     

    #      Doxygen   
    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_SOURCE_DIR}/MotionManager.h"
        "${CMAKE_CURRENT_SOURCE_DIR}/MotionManager.cpp"

        #     motion_interface_hal,      MotionManager
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    # PREDEFINED     include guards   .h ,
    #  Doxygen       .
    #      ,   #ifndef _H
    set(PREDEFINED_GUARDS 
        "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H" 
        "TRAJECTORYQUEUE_H" "IMOTION_INTERFACES_H" "ITRANSPORT_H"
        "FAKEMOTIONINTERFACE_H" "MOTION_MANAGER_H"
        #   include guards  UdpPeer.hpp, UDPTransport.h, UDPMotionInterface.h,   
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
        CALL_GRAPH             = YES #   RT-
        CALLER_GRAPH           = YES
        NAMESPACE_DIAGRAMS     = YES
        WARN_IF_UNDOCUMENTED   = YES 
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        PREDEFINED             = ${PREDEFINED_GUARDS_STRING}
        QUIET                  = YES 
    ")

    add_custom_target(docs_motion_manager COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_motion_manager
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for MotionManager..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_motion_manager' target added.")
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs_motion_manager' will not be added.")
endif()