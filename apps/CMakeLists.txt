# Apps
include_directories(${RSCL_INCLUDE_DIRS} ${VREP_DRIVER_INCLUDE_DIRS})

file(
    GLOB_RECURSE
    stop_controller_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/stop_controller_example/*
)

add_executable(stop_controller_example ${stop_controller_source_files})
set_property(TARGET stop_controller_example PROPERTY CXX_STANDARD 14)
target_link_libraries(stop_controller_example RSCL)
target_link_libraries(stop_controller_example vrep_driver)
