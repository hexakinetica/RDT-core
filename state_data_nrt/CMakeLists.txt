# CMakeLists.txt   state_data

cmake_minimum_required(VERSION 3.15)
project(StateDataTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) #  C++20
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# ---   ---
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---     ---
add_executable(StateDataTestApp
    state_data_main.cpp
    # StateData.h - header-only (   StateData.cpp,  )

    #  .cpp   
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
    # Logger.h, Units.h - header-only
)

# ---      ---
target_include_directories(StateDataTestApp PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                         #  StateData.h
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"       #  DataTypes.h, Units.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"                #  Logger.h
)

# ---   pthread,   std::jthread/std::thread    ---
#if(UNIX AND NOT APPLE) #  Linux      pthread
#    target_link_libraries(StateDataTestApp PRIVATE Threads::Threads)
#endif()


# --- Doxygen    ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "StateData Module")
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_state_data")
    set(DOXYGEN_INPUT_FILES
        "${CMAKE_CURRENT_LIST_DIR}/StateData.h"
        "${CMAKE_CURRENT_LIST_DIR}/state_data_test_main.cpp"
    )
    string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")
    set(PREDEFINED_GUARDS_SD "STATEDATA_H" "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H")
    string(REPLACE ";" " " PREDEFINED_GUARDS_SD_STRING "${PREDEFINED_GUARDS_SD}")

    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_state_data "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = ${DOXYGEN_INPUT_STRING}
        INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \
                                 \"${CMAKE_CURRENT_LIST_DIR}/../logger\"
        RECURSIVE              = NO
        EXTRACT_ALL            = YES
        GENERATE_HTML          = YES HTML_OUTPUT = html HAVE_DOT = YES CLASS_DIAGRAMS = YES
        ENABLE_PREPROCESSING   = YES MACRO_EXPANSION  = YES PREDEFINED = ${PREDEFINED_GUARDS_SD_STRING}
        QUIET                  = YES WARN_IF_UNDOCUMENTED = YES 
    ")
    add_custom_target(docs_state_data COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_state_data
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API for StateData..." VERBATIM)
    message(STATUS "Doxygen: 'docs_state_data' target added.")
else()
    message(WARNING "Doxygen not found. 'docs_state_data' target not added.")
endif()