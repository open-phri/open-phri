set(PID_WS_FOUND FALSE)

project(RobotSafetyControlLibrary)

find_package(Doxygen)
option(BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

option(BUILD_DEBUG "Build everything for debugging" OFF)

if(BUILD_DEBUG)
    set(CMAKE_BUILD_TYPE Debug)
else()
    set(CMAKE_BUILD_TYPE Release)
endif()

# Use Ccache if it is available on the system
find_program(CCACHE_FOUND ccache)
if(CCACHE_FOUND)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
endif(CCACHE_FOUND)

# Libraries
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

# Applications
add_subdirectory(${PROJECT_SOURCE_DIR}/apps)

# Unit tests
add_subdirectory(${PROJECT_SOURCE_DIR}/test)

# Bindings
# add_subdirectory(${PROJECT_SOURCE_DIR}/bindings)

# Documentation
if(BUILD_DOCUMENTATION)
    if(NOT DOXYGEN_FOUND)
        message(FATAL_ERROR "Doxygen is needed to build the documentation.")
    endif()

    set(doxyfile ${PROJECT_SOURCE_DIR}/share/Doxyfile)

    add_custom_target(doc
        COMMAND ${DOXYGEN_EXECUTABLE} ${doxyfile}
        WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/share
        COMMENT "Generating API documentation with Doxygen"
        VERBATIM)
endif()
