function(create_test test)
    PID_Component(
        TEST_APPLICATION
        NAME  ${test}
        DIRECTORY ${test}
        RUNTIME_RESOURCES robot_models
        DEPEND open-phri/open-phri pid-rpath/rpathlib catch2/catch2
    )

    run_PID_Test(
        NAME test-${test}
        COMPONENT ${test})

    set(PY_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/${test}/${test}.py")
    if(RUN_PYTHON_TESTS AND EXISTS ${PY_TEST_FILE})
        add_test(NAME "test-${test}-py" COMMAND ${PYTHON_EXECUTABLE} ${PY_TEST_FILE})
    endif()
endfunction()


create_test(default_controller)
create_test(interpolators)
# create_test(potential_field_generator)
create_test(task_space)
# create_test(proxy_generators)

create_test(constraints)
