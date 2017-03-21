find_package(Boost COMPONENTS python3 REQUIRED)
find_package(PythonLibs)

include_directories(${RSCL_INCLUDE_DIRS})
include_directories(${VREP_DRIVER_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

add_library(PyRSCL SHARED RSCL.cpp)
target_link_libraries(PyRSCL ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} RSCL vrep_driver)
set_property(TARGET PyRSCL PROPERTY CXX_STANDARD 14)
set_target_properties(PyRSCL PROPERTIES PREFIX "") # python needs a PyRSCL.so file, without prefix

configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/__init__.py COPYONLY)
