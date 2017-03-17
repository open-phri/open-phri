cmake_minimum_required (VERSION 2.6)

project(RobotSafetyControlLibrary)

# Libraries
add_subdirectory(${PROJECT_SOURCE_DIR}/src)

# Applications
add_subdirectory(${PROJECT_SOURCE_DIR}/apps)

# Unit tests
add_subdirectory(${PROJECT_SOURCE_DIR}/tests)
