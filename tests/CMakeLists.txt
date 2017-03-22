option(RUN_PYTHON_TESTS "Run also the Python version of the tests" OFF)
if(RUN_PYTHON_TESTS)
    find_package(PythonInterp REQUIRED)
endif()

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
    set_property(TARGET ${test} PROPERTY CXX_STANDARD 14)
    if(RUN_PYTHON_TESTS AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/${test}.py")
        add_test(NAME "${test}_test_py" COMMAND ${PYTHON_EXECUTABLE} "${CMAKE_CURRENT_SOURCE_DIR}/${test}.py")
    endif()
endfunction()

create_test(default_controller)
create_test(constant_generators)
create_test(add_remove_constraints)
create_test(velocity_constraint)
