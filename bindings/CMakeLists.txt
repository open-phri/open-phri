option(GEN_PYTHON_BINDINGS "Generate bindings for Python" OFF)

if(GEN_PYTHON_BINDINGS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/python)
endif()
