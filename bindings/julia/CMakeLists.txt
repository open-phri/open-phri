set(CxxWrap_DIR "$ENV{HOME}/.julia/v0.5/CxxWrap/deps/usr/lib/cmake")

find_package(CxxWrap)

include_directories(${RSCL_INCLUDE_DIRS})
include_directories(${VREP_DRIVER_INCLUDE_DIRS})
include_directories(${Boost_INCLUDE_DIRS})
include_directories(${PYTHON_INCLUDE_DIRS})

add_definitions(-DJULIA_ENABLE_THREADING)

add_library(JlRSCL SHARED RSCL.cpp)
target_link_libraries(JlRSCL CxxWrap::cxx_wrap RSCL vrep_driver)
set_property(TARGET JlRSCL PROPERTY CXX_STANDARD 14)
