cmake_minimum_required (VERSION 2.6)

project(AdaptiveDampingFramework)

FIND_PACKAGE(Eigen3)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

include_directories(include)
include_directories( ${EIGEN3_INCLUDE_DIRS} )

# Library
file(
        GLOB_RECURSE
        lib_source_files
        src/*
)

add_library(adaptive-damping ${lib_source_files})
set_property(TARGET adaptive-damping PROPERTY CXX_STANDARD 14)

# Apps
file(
        GLOB_RECURSE
        stop_controller_source_files
        apps/stop_controller_example/*
)

add_executable(stop_controller_example ${stop_controller_source_files})
target_link_libraries(stop_controller_example adaptive-damping)
