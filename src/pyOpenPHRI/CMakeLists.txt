find_package(Boost COMPONENTS python3 REQUIRED)
find_package(PythonLibs REQUIRED)

include_directories(${OpenPHRI_INCLUDE_DIRS})
include_directories(${VREP_DRIVER_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

file(
    GLOB_RECURSE
    openphri_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/*.cpp
)

add_library(PyOpenPHRI SHARED ${openphri_source_files})
target_link_libraries(PyOpenPHRI ${Boost_LIBRARIES} ${PYTHON_LIBRARIES} OpenPHRI vrep_driver)
set_property(TARGET PyOpenPHRI PROPERTY CXX_STANDARD 14)
set_target_properties(PyOpenPHRI PROPERTIES PREFIX "") # python needs a PyOpenPHRI.so file, without prefix

configure_file(__init__.py ${CMAKE_CURRENT_BINARY_DIR}/__init__.py COPYONLY)
