# Apps
include_directories(${RSCL_INCLUDE_DIRS} ${VREP_DRIVER_INCLUDE_DIRS})

function(add_application app)
    file(GLOB_RECURSE app_source_files ${CMAKE_CURRENT_SOURCE_DIR}/${app}/*)
    add_executable(${app} ${app_source_files})
    set_property(TARGET ${app} PROPERTY CXX_STANDARD 14)
    target_link_libraries(${app} RSCL)
    target_link_libraries(${app} vrep_driver)
endfunction()

add_application(stop_constraint_example)
add_application(velocity_constraint_example)
add_application(obstacle_avoidance_example)
