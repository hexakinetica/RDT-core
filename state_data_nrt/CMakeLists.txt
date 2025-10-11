# ============================================================================
# CMakeLists.txt — state_data
# ============================================================================
cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(StateDataTest
  VERSION 0.1.0
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(STATEDATA_STRICT "Enable strict warnings and treat warnings as errors" ON)
option(STATEDATA_ENABLE_IPO "Enable interprocedural optimization in Release" ON)

# ============================================================================
# Dependencies
# - Prefer reusing an existing 'data_types_rdt' target from the parent project.
# - If it doesn't exist, create a minimal local fallback from DataTypes.cpp.
# ============================================================================
if (TARGET data_types_rdt)
  message(STATUS "Using existing target: data_types_rdt")
else()
  add_library(data_types_rdt STATIC
    "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt/DataTypes.cpp"
  )
  target_include_directories(data_types_rdt
    PUBLIC
      "${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt"
  )
  target_compile_features(data_types_rdt PUBLIC cxx_std_20)
endif()

# ============================================================================
# Library: state_data_nrt (header-only, exported include path)
# ============================================================================
add_library(state_data_nrt INTERFACE)
target_include_directories(state_data_nrt
  INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"           # for StateData.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger" # for Logger.h if included by headers
)
target_compile_features(state_data_nrt INTERFACE cxx_std_20)
target_link_libraries(state_data_nrt
  INTERFACE
    data_types_rdt
)

# ============================================================================
# Executable (test app)
# ============================================================================
add_executable(StateDataTestApp
  state_data_main.cpp
)

target_compile_features(StateDataTestApp PRIVATE cxx_std_20)

target_link_libraries(StateDataTestApp
  PRIVATE
    state_data_nrt
    data_types_rdt
)

target_include_directories(StateDataTestApp
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
    "${CMAKE_CURRENT_LIST_DIR}/../logger"
)

# ============================================================================
# Warnings (compiler-aware) + optional -Werror
# ============================================================================
if (MSVC)
  target_compile_options(StateDataTestApp PRIVATE /permissive- /W4)
  if (STATEDATA_STRICT)
    target_compile_options(StateDataTestApp PRIVATE /WX)
  endif()
else()
  target_compile_options(StateDataTestApp PRIVATE -Wall -Wextra -Wpedantic)
  if (STATEDATA_STRICT)
    target_compile_options(StateDataTestApp PRIVATE -Werror)
  endif()
endif()

# ============================================================================
# IPO/LTO (optional)
# ============================================================================
if (STATEDATA_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET StateDataTestApp PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    # If data_types_rdt is ours (non-IMPORTED), enable IPO as well
    if (TARGET data_types_rdt)
      get_target_property(_dt_imported data_types_rdt IMPORTED)
      if (NOT _dt_imported)
        set_property(TARGET data_types_rdt PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
      endif()
    endif()
  else()
    message(STATUS "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ============================================================================
# Doxygen (optional)
# ============================================================================
find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_NAME "StateData Module")
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_state_data")
  set(DOXYGEN_INPUT_FILES
    "${CMAKE_CURRENT_LIST_DIR}/StateData.h"
    "${CMAKE_CURRENT_LIST_DIR}/state_data_main.cpp"
  )
  string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

  # Predefined guards to help Doxygen parse headers without full transitive context
  set(PREDEFINED_GUARDS_SD "STATEDATA_H" "RDT_UNITS_H" "DATATYPES_H" "LOGGER_H")
  string(REPLACE ";" " " PREDEFINED_GUARDS_SD_STRING "${PREDEFINED_GUARDS_SD}")

  set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_state_data")
  file(WRITE "${_doxyfile}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
INPUT                  = ${DOXYGEN_INPUT_STRING}
INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \"${CMAKE_CURRENT_LIST_DIR}/../logger\"
RECURSIVE              = NO
EXTRACT_ALL            = YES
GENERATE_HTML          = YES
HTML_OUTPUT            = html
HAVE_DOT               = YES
CLASS_DIAGRAMS         = YES
ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
PREDEFINED             = ${PREDEFINED_GUARDS_SD_STRING}
QUIET                  = YES
WARN_IF_UNDOCUMENTED   = YES
")

  add_custom_target(docs_state_data
    COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating API for StateData..."
    VERBATIM
  )
  message(STATUS "Doxygen: 'docs_state_data' target added.")
else()
  message(STATUS "Doxygen not found. 'docs_state_data' target not added.")
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "Config summary (StateData):")
message(STATUS "  C++ Standard:       20")
message(STATUS "  Strict mode:        ${STATEDATA_STRICT}")
message(STATUS "  IPO enabled:        ${STATEDATA_ENABLE_IPO}")
