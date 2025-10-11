# ============================================================================
# CMakeLists.txt — interpolator_nrt
# ============================================================================
cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(InterpolatorNRT
  VERSION 0.1.0
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(INTERPOLATOR_STRICT "Enable strict warnings and treat warnings as errors" ON)
option(INTERPOLATOR_ENABLE_IPO "Enable interprocedural optimization in Release" ON)

# ============================================================================
# Dependencies
# - Eigen3 (header-only)
# - Reuse sibling targets if available:
#     data_types_rdt, logger, trajectory_queue_lf, motion_interface_hal
# - Provide a minimal local fallback for data_types_rdt when missing.
# ============================================================================
find_package(Eigen3 3.4 REQUIRED NO_MODULE)

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

if (TARGET logger)
  message(STATUS "Using existing target: logger")
else()
  add_library(logger INTERFACE)
  target_include_directories(logger INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../logger")
  target_compile_features(logger INTERFACE cxx_std_20)
endif()

if (TARGET trajectory_queue_lf)
  message(STATUS "Using existing target: trajectory_queue_lf")
else()
  add_library(trajectory_queue_lf INTERFACE)
  target_include_directories(trajectory_queue_lf INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../trajectory_queue_lf")
  target_compile_features(trajectory_queue_lf INTERFACE cxx_std_20)
endif()

if (TARGET motion_interface_hal)
  message(STATUS "Using existing target: motion_interface_hal")
else()
  # Header-only headers are still consumable without sources
  add_library(motion_interface_hal INTERFACE)
  target_include_directories(motion_interface_hal INTERFACE "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal")
  target_compile_features(motion_interface_hal INTERFACE cxx_std_20)
endif()

# ============================================================================
# Library: interpolator_nrt
# ============================================================================
add_library(interpolator_nrt STATIC
  TrajectoryInterpolator.cpp
  MotionProfile.cpp
)
target_include_directories(interpolator_nrt
  PUBLIC
    "${CMAKE_CURRENT_LIST_DIR}"                   # headers of this module
    "${CMAKE_CURRENT_LIST_DIR}/../motion_interface_hal" # for IMotionInterfaces.* if included by headers
)
target_compile_features(interpolator_nrt PUBLIC cxx_std_20)
target_link_libraries(interpolator_nrt
  PUBLIC
    Eigen3::Eigen
    data_types_rdt
    logger
    trajectory_queue_lf
    motion_interface_hal
)

# ============================================================================
# Executable: InterpolatorTestApp
# ============================================================================
add_executable(InterpolatorTestApp
  interpolator_main.cpp
  # KdlKinematicSolver_mock.h is header-only and included from sources as needed
)
target_compile_features(InterpolatorTestApp PRIVATE cxx_std_20)
target_link_libraries(InterpolatorTestApp
  PRIVATE
    interpolator_nrt
)

# ============================================================================
# Warnings (compiler-aware) + optional -Werror
# ============================================================================
if (MSVC)
  target_compile_options(InterpolatorTestApp PRIVATE /permissive- /W4)
  if (INTERPOLATOR_STRICT)
    target_compile_options(InterpolatorTestApp PRIVATE /WX)
  endif()
else()
  target_compile_options(InterpolatorTestApp PRIVATE -Wall -Wextra -Wpedantic)
  if (INTERPOLATOR_STRICT)
    target_compile_options(InterpolatorTestApp PRIVATE -Werror)
  endif()
endif()

# ============================================================================
# IPO/LTO (optional)
# ============================================================================
if (INTERPOLATOR_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET interpolator_nrt    PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    set_property(TARGET InterpolatorTestApp PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    # Enable for local (non-imported) fallbacks as well
    foreach(dep IN ITEMS data_types_rdt)
      if (TARGET ${dep})
        get_target_property(_is_imported ${dep} IMPORTED)
        if (NOT _is_imported)
          set_property(TARGET ${dep} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
        endif()
      endif()
    endforeach()
  else()
    message(STATUS "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ============================================================================
# Doxygen (optional)
# ============================================================================
find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_NAME "InterpolatorNRT Module")
  set(DOXYGEN_OUTPUT_DIR   "${CMAKE_BINARY_DIR}/docs/api_interpolator_nrt")

  set(DOXYGEN_INPUT_FILES
    "${CMAKE_CURRENT_LIST_DIR}/MotionProfile.h"
    "${CMAKE_CURRENT_LIST_DIR}/MotionProfile.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/TrajectoryInterpolator.h"
    "${CMAKE_CURRENT_LIST_DIR}/TrajectoryInterpolator.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/interpolator_main.cpp"
    "${CMAKE_CURRENT_LIST_DIR}/KdlKinematicSolver_mock.h"
  )
  string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

  set(PREDEFINED_GUARDS
    "MOTIONPROFILE_H"
    "TRAJECTORYINTERPOLATOR_H"
    "KDL_KINEMATIC_SOLVER_MOCK_H"
    "RDT_UNITS_H"
    "DATATYPES_H"
    "LOGGER_H"
  )
  string(REPLACE ";" " " PREDEFINED_GUARDS_STRING "${PREDEFINED_GUARDS}")

  set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_interpolator_nrt")
  file(WRITE "${_doxyfile}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
PROJECT_NUMBER         = \"${PROJECT_VERSION}\"
OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
INPUT                  = ${DOXYGEN_INPUT_STRING}
INCLUDE_PATH           = \"${CMAKE_CURRENT_LIST_DIR}/../data_types_rdt\" \"${CMAKE_CURRENT_LIST_DIR}/../logger\"
RECURSIVE              = NO
EXTRACT_ALL            = YES
EXTRACT_PRIVATE        = NO
EXTRACT_STATIC         = YES
GENERATE_HTML          = YES
HTML_OUTPUT            = html
HAVE_DOT               = YES
UML_LOOK               = YES
CLASS_DIAGRAMS         = YES
CALL_GRAPH             = YES
CALLER_GRAPH           = YES
ENABLE_PREPROCESSING   = YES
MACRO_EXPANSION        = YES
PREDEFINED             = ${PREDEFINED_GUARDS_STRING}
QUIET                  = YES
WARN_IF_UNDOCUMENTED   = NO
")

  add_custom_target(docs_interpolator_nrt
    COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating API documentation for InterpolatorNRT..."
    VERBATIM
  )
  message(STATUS "Doxygen: 'docs_interpolator_nrt' target added.")
else()
  message(STATUS "Doxygen not found; 'docs_interpolator_nrt' target will not be generated.")
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "Config summary (InterpolatorNRT):")
message(STATUS "  C++ Standard:       20")
message(STATUS "  Strict mode:        ${INTERPOLATOR_STRICT}")
message(STATUS "  IPO enabled:        ${INTERPOLATOR_ENABLE_IPO}")
