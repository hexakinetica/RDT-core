# CMakeLists.txt   spatial_math ( FrameTransformer)

cmake_minimum_required(VERSION 3.15)
project(SpatialMathTest LANGUAGES CXX) #     

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---   ---
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic) # -Werror    CI/
endif()

# ---     ---
add_executable(FrameTransformerTestApp #    
    frame_transformer_main.cpp
    # FrameTransformer.h - header-only,  

    #  .cpp   ,    main  FrameTransformer (    .cpp)
    # CMAKE_CURRENT_LIST_DIR -  /path/to/RobotControl_MVP/spatial_math/
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    # Logger.h - header-only
)

# ---      ---
target_include_directories(FrameTransformerTestApp PUBLIC
    #     (FrameTransformer.h)
    "${CMAKE_CURRENT_LIST_DIR}"

    #    
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"                #  Logger.h

    #   Eigen3 (     libs   )
    "${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0"
)

# ---    (Eigen  header-only) ---
#   Eigen  :
# find_package(Eigen3 3.3 REQUIRED) # NO_MODULE  ,     
# target_link_libraries(FrameTransformerTestApp PRIVATE Eigen3::Eigen)


# --- Doxygen    ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "SpatialMath (FrameTransformer) Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_spatial_math")

    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/FrameTransformer.h"
        "${CMAKE_CURRENT_LIST_DIR}/frame_transformer_main.cpp"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

    set(PREDEFINED_GUARDS_SM
        "FRAMETRANSFORMER_H"
        "RDT_UNITS_H"           #  Units.h
        "DATATYPES_H"           #  DataTypes.h
        "LOGGER_H"              #  Logger.h
    )
    string(REPLACE ";" " " PREDEFINED_GUARDS_SM_STRING "${PREDEFINED_GUARDS_SM}")

    file(WRITE ${CMAKE_BINARY_DIR}/Doxyfile_spatial_math "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../logger\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../../libs/eigen-3.4.0\"
        RECURSIVE              = NO
        EXTRACT_ALL            = YES
        GENERATE_HTML          = YES HTML_OUTPUT = html HAVE_DOT = YES CLASS_DIAGRAMS = YES
        ENABLE_PREPROCESSING   = YES MACRO_EXPANSION  = YES PREDEFINED = ${PREDEFINED_GUARDS_SM_STRING}
        QUIET                  = YES
        WARN_IF_UNDOCUMENTED   = YES 
    ")
    add_custom_target(docs_spatial_math COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_spatial_math
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation for SpatialMath (FrameTransformer)..." VERBATIM)
    message(STATUS "Doxygen: 'docs_spatial_math' target added.")
else()
    message(WARNING "Doxygen not found. API documentation target 'docs_spatial_math' will not be added.")
endif()