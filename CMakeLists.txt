cmake_minimum_required (VERSION 2.6)

project(RobotSafetyControlLibrary)

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
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)

# Bindings
add_subdirectory(${PROJECT_SOURCE_DIR}/bindings)
