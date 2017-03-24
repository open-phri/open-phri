option(GEN_PYTHON_BINDINGS "Generate bindings for Python" OFF)
option(GEN_JULIA_BINDINGS "Generate bindings for Julia" OFF)

if(GEN_PYTHON_BINDINGS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/python)
endif()

if(GEN_JULIA_BINDINGS)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/julia)
endif()
