cmake_minimum_required(VERSION 3.15)
project(InterpolatorNRT LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) # , C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) #  

# ---   ---
#    if(MSVC) ... else() ... endif()   -
#add_compile_options(-Wall -Wextra -Wpedantic -Werror) #  GCC/Clang



# ---     Interpolator ---
add_executable(InterpolatorTestApp
    #   
    interpolator_main.cpp
    TrajectoryInterpolator.cpp
    MotionProfile.cpp
    # KdlKinematicSolver_mock.h - header-only,    interpolator_main.cpp

    #  .cpp    (..     )
    #    CMAKE_CURRENT_LIST_DIR (  CMakeLists.txt)
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    # Logger.h - header-only
    # TrajectoryQueue.h - header-only
    # IMotionInterfaces.h - header-only (  .cpp)
    # ITransport.h - header-only (  .cpp)
    # FakeMotionInterface.cpp - ,  interpolator_main.cpp  
    # UdpPeer.hpp - header-only
    # UDPTransport.cpp - ,  
    # UDPMotionInterface.cpp - ,  
)

# ---      ---
# CMAKE_CURRENT_LIST_DIR -  /path/to/RobotControl_MVP/interpolator_nrt/
target_include_directories(InterpolatorTestApp PUBLIC
    #    
    "${CMAKE_CURRENT_LIST_DIR}"

    #    
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"                #  Logger.h
    "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf"   #  TrajectoryQueue.h
    # IMotionInterfaces  FakeMotionInterface       ,
    #  KdlKinematicSolver_mock       .
    #     (,   TrajectoryPoint   motion_interface):
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal"  #  IMotionInterfaces.h, FakeMotionInterface.h, etc.
    "${CMAKE_CURRENT_SOURCE_DIR}/../../libs/eigen-3.4.0/"
    #  Eigen3
    ${EIGEN3_INCLUDE_DIRS}
)

# ---    ( ) ---
# target_link_libraries(InterpolatorTestApp PRIVATE ${EIGEN3_LIBRARIES}) #  Eigen3 - header-only,   

# --- Doxygen    ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "InterpolatorNRT Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_interpolator_nrt") #  

    # INPUT     
    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/MotionProfile.h"
        "${CMAKE_CURRENT_LIST_DIR}/MotionProfile.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryInterpolator.h"
        "${CMAKE_CURRENT_LIST_DIR}/TrajectoryInterpolator.cpp"
        "${CMAKE_CURRENT_LIST_DIR}/interpolator_main.cpp" #  main   
        "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver_mock.h" #  mock 
        # :    .h/.cpp   ,       .
        # Doxygen     #include,     PREDEFINED   Doxygen    .
        #    ,             ,
        #   .h     INPUT,      .
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    # PREDEFINED    include guards    ,
    #     ( ),  Doxygen    .
    #         include-  TAGFILES  INPUT ( Doxygen)
    #  ,    PREDEFINED  .
    #   ,  include guards    ,   .
    set(PREDEFINED_GUARDS
        "MOTIONPROFILE_H"           #  MotionProfile.h
        "TRAJECTORYINTERPOLATOR_H"  #  TrajectoryInterpolator.h
        "KDL_KINEMATIC_SOLVER_MOCK_H" #  KdlKinematicSolver_mock.h
        # Guards   :
        "RDT_UNITS_H"               #  Units.h
        "DATATYPES_H"               #  DataTypes.h
        "LOGGER_H"                  #  Logger.h
        #  ,      .h 
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_STRING "${PREDEFINED_GUARDS}")

    # Doxyfile   
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_interpolator_nrt "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        PROJECT_NUMBER         = \"${PROJECT_VERSION}\" #    CMAKE_PROJECT_VERSION
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        #  Doxygen,     
        INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../logger\"
                                 #     include,  
        RECURSIVE              = NO #     INPUT 
        EXTRACT_ALL            = YES
        EXTRACT_PRIVATE        = NO #  YES,     
        EXTRACT_STATIC         = YES
        GENERATE_HTML          = YES
        HTML_OUTPUT            = html
        HAVE_DOT               = YES #   NO,   Graphviz
        UML_LOOK               = YES #  HAVE_DOT = YES
        CLASS_DIAGRAMS         = YES
        CALL_GRAPH             = YES
        CALLER_GRAPH           = YES
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        PREDEFINED             = ${PREDEFINED_GUARDS_STRING}
        QUIET                  = YES
        WARN_IF_UNDOCUMENTED   = NO #   YES,      
    ")

    add_custom_target(docs_interpolator_nrt COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_interpolator_nrt
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for InterpolatorNRT..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs_interpolator_nrt' target added.")
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs_interpolator_nrt' will not be added.")
endif()