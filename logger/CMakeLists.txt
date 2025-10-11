# ============================================================================
# CMakeLists.txt — logger
# ============================================================================
cmake_minimum_required(VERSION 3.18)

# ============================================================================
# Project
# ============================================================================
project(LoggerTest
  VERSION 0.1.0
  LANGUAGES CXX
)

# ============================================================================
# Options
# ============================================================================
option(LOGGER_STRICT "Enable strict warnings and treat warnings as errors" ON)
option(LOGGER_ENABLE_IPO "Enable interprocedural optimization in Release" ON)

# ============================================================================
# Library (header-only)
# ============================================================================
add_library(logger INTERFACE)
target_include_directories(logger
  INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}   # for Logger.h
)
target_compile_features(logger INTERFACE cxx_std_20)

# ============================================================================
# Executable
# ============================================================================
add_executable(LoggerTest
  logger_main.cpp
)
target_compile_features(LoggerTest PRIVATE cxx_std_20)
target_link_libraries(LoggerTest PRIVATE logger)

# ============================================================================
# Warnings (compiler-aware) + optional -Werror
# ============================================================================
if (MSVC)
  target_compile_options(LoggerTest PRIVATE /permissive- /W4)
  if (LOGGER_STRICT)
    target_compile_options(LoggerTest PRIVATE /WX)
  endif()
else()
  target_compile_options(LoggerTest PRIVATE -Wall -Wextra -Wpedantic)
  if (LOGGER_STRICT)
    target_compile_options(LoggerTest PRIVATE -Werror)
  endif()
endif()

# ============================================================================
# IPO/LTO (optional)
# ============================================================================
if (LOGGER_ENABLE_IPO)
  include(CheckIPOSupported)
  check_ipo_supported(RESULT ipo_ok OUTPUT ipo_msg)
  if (ipo_ok)
    set_property(TARGET LoggerTest PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
  else()
    message(STATUS "IPO not supported: ${ipo_msg}")
  endif()
endif()

# ============================================================================
# Doxygen (optional)
# ============================================================================
find_package(Doxygen QUIET)
if (DOXYGEN_FOUND)
  set(DOXYGEN_PROJECT_NAME "Logger Module")
  set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_logger")
  set(DOXYGEN_INPUT_FILES
    "${CMAKE_CURRENT_LIST_DIR}/Logger.h"
    "${CMAKE_CURRENT_LIST_DIR}/logger_main.cpp"
  )
  string(REPLACE ";" " " DOXYGEN_INPUT_STRING "${DOXYGEN_INPUT_FILES}")

  set(_doxyfile "${CMAKE_CURRENT_BINARY_DIR}/Doxyfile_logger")
  file(WRITE "${_doxyfile}" "
PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
PROJECT_NUMBER         = \"${PROJECT_VERSION}\"
OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
INPUT                  = ${DOXYGEN_INPUT_STRING}
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

  add_custom_target(docs_logger
    COMMAND ${DOXYGEN_EXECUTABLE} "${_doxyfile}"
    WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
    COMMENT "Generating API documentation with Doxygen for Logger..."
    VERBATIM
  )
  message(STATUS "Doxygen: 'docs_logger' target added. Use 'cmake --build . --target docs_logger'")
else()
  message(STATUS "Doxygen not found; 'docs_logger' target will not be generated.")
endif()

# ============================================================================
# Summary
# ============================================================================
message(STATUS "Config summary (Logger):")
message(STATUS "  C++ Standard:       20")
message(STATUS "  Strict mode:        ${LOGGER_STRICT}")
message(STATUS "  IPO enabled:        ${LOGGER_ENABLE_IPO}")
