cmake_minimum_required(VERSION 3.15) # Using a reasonably modern version
project(DataTypesTest LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20) # Most of the code uses C++17 features (like structured bindings if we add them)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF) # Good practice


add_executable(LoggerTest
    logger_main.cpp
)

# Optional: Add compiler warnings (good practice)
target_compile_options(LoggerTest PRIVATE -Wall -Wextra -Wpedantic -Werror)



# --- Doxygen Documentation Target ---
find_package(Doxygen)
if(DOXYGEN_FOUND)
    set(DOXYGEN_PROJECT_NAME "Logger Module") # More specific name for Doxy
    set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs/api_datatypes")
    set(DOXYGEN_INPUT_DIR "${CMAKE_CURRENT_SOURCE_DIR}") # Assuming .h and .cpp are here

    # Create a Doxyfile on the fly or use a Doxyfile.in template
    # For simplicity, creating one here:
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile "
        PROJECT_NAME           = \"${DOXYGEN_PROJECT_NAME}\"
        PROJECT_NUMBER         = \"${PROJECT_VERSION}\" # If PROJECT_VERSION is set
        OUTPUT_DIRECTORY       = \"${DOXYGEN_OUTPUT_DIR}\"
        INPUT                  = \"${DOXYGEN_INPUT_DIR}/Units.h\" \"${DOXYGEN_INPUT_DIR}/DataTypes.h\" \"${DOXYGEN_INPUT_DIR}/DataTypes.cpp\"
        RECURSIVE              = NO # We listed files explicitly
        EXTRACT_ALL            = YES
        EXTRACT_PRIVATE        = NO
        EXTRACT_STATIC         = YES
        GENERATE_HTML          = YES
        HTML_OUTPUT            = html
        HAVE_DOT               = YES # Set to NO if no Graphviz
        UML_LOOK               = YES # For better UML diagrams if HAVE_DOT is YES
        CLASS_DIAGRAMS         = YES
        NAMESPACE_DIAGRAMS     = YES
        WARN_IF_UNDOCUMENTED   = YES # Good for checking coverage
        ENABLE_PREPROCESSING   = YES
        MACRO_EXPANSION        = YES
        PREDEFINED             = \"RDT_UNITS_H\" \"DATATYPES_H\" # Helps Doxygen parse includes
    ")

    add_custom_target(docs
        COMMAND ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen for DataTypes..."
        VERBATIM
    )
    message(STATUS "Doxygen: 'docs' target added. Run 'cmake --build . --target docs' to generate.")
else()
    message(WARNING "Doxygen: Not found. API documentation target 'docs' will not be added.")
endif()