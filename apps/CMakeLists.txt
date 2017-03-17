# Apps
include_directories(${RSCL_INCLUDE_DIRS} ${VREP_REMOTE_API_INCLUDE_DIRS})

file(
    GLOB_RECURSE
    stop_controller_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/stop_controller_example/*
)

add_executable(stop_controller_example ${stop_controller_source_files})
target_link_libraries(stop_controller_example RSCL)
