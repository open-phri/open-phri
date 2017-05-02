if(RUN_PYTHON_TESTS)
    find_package(PythonInterp REQUIRED)
endif()

enable_testing()

add_custom_target(run-tests
    COMMAND ctest
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/build/test
)

include_directories(${RSCL_INCLUDE_DIRS} ${VREP_REMOTE_API_INCLUDE_DIRS})

function(create_test test)
    add_executable(${test} "${test}/${test}.cpp")
    add_test("test-${test}" ${test})
    target_link_libraries(${test} RSCL)
    set_property(TARGET ${test} PROPERTY CXX_STANDARD 14)
    set(PY_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${test}/${test}.py")
    if(RUN_PYTHON_TESTS AND EXISTS ${PY_TEST_FILE})
        add_test(NAME "test-${test}-py" COMMAND ${PYTHON_EXECUTABLE} ${PY_TEST_FILE})
    endif()
endfunction()

create_test(default_controller)
create_test(proxy_generators)
create_test(add_remove_constraints)
create_test(velocity_constraint)
create_test(power_constraint)
create_test(stop_constraint)
create_test(potential_field_generator)
create_test(interpolators)
