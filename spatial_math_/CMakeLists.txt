# ============================================================================
# CMakeLists.txt — spatial_math (FrameTransformer)
# ============================================================================
cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(SpatialMathTest
  VERSION 0.1.0
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(SPATIALMATH_STRICT "Enable strict warnings and treat warnings as errors" ON)
option(SPATIALMATH_ENABLE_IPO "Enable interprocedural optimization in Release" ON)

# ============================================================================
# Dependencies
# - Eigen3 (header-only) is required.
# - Prefer reusing 'data_types_rdt' and 'logger' targets from the parent project.
#   If 'data_types_rdt' is missing, create a minimal local fallback from DataTypes.cpp.
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

# ============================================================================
# Library: spatial_math_ (header-only FrameTransformer)
# ============================================================================
add_library(spatial_math_ INTERFACE)
target_include_directories(spatial_math_
  INTERFACE
    "${CMAKE_CURRENT_LIST_DIR}"             # for FrameTransformer.h
    "${CMAKE_CURRENT_LIST_DIR}/../logger"   # for Logger.h used from headers
)
target_link_libraries(spatial_math_
  INTERFACE
    Eigen3::Eigen
    data_types_rdt
    logger
)
target_compile_features(spatial_math_ INTERFACE cxx_std_20)

# ============================================================================
# Executable: FrameTransformerTestApp
# ============================================================================
add_executable(FrameTransformerTestApp
  frame_transformer_main.cpp
)
target_compile_features(FrameTransformerTestApp PRIVATE cxx_std_20)
target_link_libraries(FrameTransformerTestApp
  PRIVATE
    spatial_math_
)
target_include_directories(FrameTransformerTestApp
  PRIVATE
    "${CMAKE_CURRENT_LIST_DIR}"
)

# ============================================================================
# Warnings (compiler-aware) + optional -Werror
# ============================================================================
if (MSVC)
  target_compile_options(FrameTransformerTestApp PRIVATE /permissive- /W4)
  if (SPATIALMATH_STRICT)
    target_compile_options(FrameTransformerTestApp PRIVATE /WX)
  endif()
else()
  target_compile_options(FrameTransformerTestApp PRIVATE -Wall -Wextra -Wpedantic)
  if (SPATIALMATH_STRICT)
    target_compile_options(FrameTransformerTestApp PRIVATE -Werror)
  endif()
endif()

# ============================================================================
# IPO/LTO (optional)
# ============================================================================
if (SPATIALMATH_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET FrameTransformerTestApp PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    # data_types_rdt may be local and non-IMPORTED; enable IPO if so
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
  set(DOXYGEN_PROJECT_NAME "SpatialMath (FrameTransformer) Module")
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_spatial_math")

  set(DOXYGEN_INPUT_FILES
    "${CMAKE_CURRENT_LIST_DIR}/FrameTransformer.h"
    "${CMAKE_CURRENT_LIST_DIR}/frame_transformer_main.cpp"
  )
  string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

  # Predefined guards to help Doxygen parse headers
  set(PREDEFINED_GUARDS_SM
    "FRAMETRANSFORMER_H"
    "RDT_UNITS_H"
    "DATATYPES_H"
    "LOGGER_H"
  )
  string(REPLACE ";" " " PREDEFINED_GUARDS_SM_STRING "${PREDEFINED_GUARDS_SM}")

  set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_spatial_math")
  file(WRITE "${_doxyfile}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
PROJECT_NUMBER         = \"${PROJECT_VERSION}\"
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
PREDEFINED             = ${PREDEFINED_GUARDS_SM_STRING}
QUIET                  = YES
WARN_IF_UNDOCUMENTED   = YES
")

  add_custom_target(docs_spatial_math
    COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating API documentation for SpatialMath (FrameTransformer)..."
    VERBATIM
  )
  message(STATUS "Doxygen: 'docs_spatial_math' target added.")
else()
  message(STATUS "Doxygen not found; 'docs_spatial_math' target will not be generated.")
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "Config summary (SpatialMath / FrameTransformer):")
message(STATUS "  C++ Standard:       20")
message(STATUS "  Strict mode:        ${SPATIALMATH_STRICT}")
message(STATUS "  IPO enabled:        ${SPATIALMATH_ENABLE_IPO}")
