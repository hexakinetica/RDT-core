cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(DataTypesTest
  VERSION 0.1.0
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(DATATYPES_STRICT "Enable strict warnings and treat warnings as errors" ON)
option(DATATYPES_ENABLE_IPO "Enable interprocedural optimization in Release" ON)

# ============================================================================
# Dependencies (local fallback for data_types_rdt)
# If the parent project already defines 'data_types_rdt', we reuse it.
# Otherwise, we create a minimal local library from DataTypes.cpp.
# ============================================================================
if (TARGET data_types_rdt)
  message(STATUS "Using existing target: data_types_rdt")
else()
  add_library(data_types_rdt STATIC
    DataTypes.cpp
  )
  target_include_directories(data_types_rdt
    PUBLIC
      ${CMAKE_CURRENT_SOURCE_DIR}
  )
  target_compile_features(data_types_rdt PUBLIC cxx_std_20)
endif()

# ============================================================================
# Executable
# ============================================================================
add_executable(DataTypesTest
  data_types_main.cpp
)

target_compile_features(DataTypesTest PRIVATE cxx_std_20)

target_link_libraries(DataTypesTest
  PRIVATE
    data_types_rdt
)

target_include_directories(DataTypesTest
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# ============================================================================
# Warnings (compiler-aware) + optional -Werror
# ============================================================================
if (MSVC)
  target_compile_options(DataTypesTest PRIVATE /permissive- /W4)
  if (DATATYPES_STRICT)
    target_compile_options(DataTypesTest PRIVATE /WX)
  endif()
else()
  target_compile_options(DataTypesTest PRIVATE -Wall -Wextra -Wpedantic)
  if (DATATYPES_STRICT)
    target_compile_options(DataTypesTest PRIVATE -Werror)
  endif()
endif()

# ============================================================================
# IPO/LTO (optional)
# ============================================================================
if (DATATYPES_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET DataTypesTest PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
    set_property(TARGET data_types_rdt PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
  else()
    message(STATUS "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ============================================================================
# Doxygen (optional)
# ============================================================================
find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_NAME "Robot DataTypes Module")
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_datatypes")
  set(DOXYGEN_INPUT_DIR  "${CMAKE_CURRENT_SOURCE_DIR}")

  # Generate a simple Doxyfile on the fly
  set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile")
  file(WRITE "${_doxyfile}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
PROJECT_NUMBER         = \"${PROJECT_VERSION}\"
OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
INPUT                  = \"${DOXYGEN_INPUT_DIR}/Units.h\" \"${DOXYGEN_INPUT_DIR}/DataTypes.h\" \"${DOXYGEN_INPUT_DIR}/DataTypes.cpp\"
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
PREDEFINED             = \"RDT_UNITS_H\" \"DATATYPES_H\"
")

  add_custom_target(docs
    COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating API documentation with Doxygen for DataTypes..."
    VERBATIM
  )
  message(STATUS "Doxygen: 'docs' target added. Use 'cmake --build . --target docs'")
else()
  message(STATUS "Doxygen not found; 'docs' target will not be generated.")
endif()

# ============================================================================
# Nice QoL messages
# ============================================================================
message(STATUS "Config summary:")
message(STATUS "  C++ Standard:       20")
message(STATUS "  Strict mode:        ${DATATYPES_STRICT}")
message(STATUS "  IPO enabled:        ${DATATYPES_ENABLE_IPO}")
