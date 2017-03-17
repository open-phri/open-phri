enable_testing()

add_custom_target(run-tests
    COMMAND ctest
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/tests
)

include_directories(${RSCL_INCLUDE_DIRS} ${VREP_REMOTE_API_INCLUDE_DIRS})

function(create_test test)
    add_executable(${test} "${test}.cpp")
    add_test("${test}_test" ${test})
    target_link_libraries(${test} RSCL)
endfunction()

create_test(default_controller)
create_test(constant_generators)
create_test(add_remove_constraints)
create_test(velocity_constraint)
