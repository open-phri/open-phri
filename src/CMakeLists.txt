###					Find libraries					###
find_package(Eigen3)
find_package(Threads)


###					RSCL library					###
set(RSCL_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include;${EIGEN3_INCLUDE_DIRS}" CACHE INTERNAL "")
include_directories(${RSCL_INCLUDE_DIRS})
file(
    GLOB_RECURSE
    rscl_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/RSCL/*
)

add_library(RSCL SHARED ${rscl_source_files})
set_property(TARGET RSCL PROPERTY CXX_STANDARD 14)


###				vrep_remote_api library				###
# Set V-REP flags according to the platform
if(APPLE)
	set(VREP_CFLAGS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;__APPLE__")
elseif(UNIX)
	set(VREP_CFLAGS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;__linux")
elseif(WIN32)
	set(VREP_CFLAGS "NON_MATLAB_PARSING;MAX_EXT_API_CONNECTIONS=255;_WIN32")
endif()

set(VREP_REMOTE_API_INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/include/vrep_remote_api CACHE INTERNAL "")
include_directories(${VREP_REMOTE_API_INCLUDE_DIRS})
file(
    GLOB_RECURSE
    remote_api_lib_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/vrep_remote_api/*
)

add_library(vrep_remote_api SHARED ${remote_api_lib_source_files})
target_link_libraries(vrep_remote_api ${CMAKE_THREAD_LIBS_INIT})
target_compile_definitions(vrep_remote_api PRIVATE ${VREP_CFLAGS})


###				vrep_driver library				###
set(VREP_DRIVER_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include;${RSCL_INCLUDE_DIRS};${VREP_REMOTE_API_INCLUDE_DIRS}" CACHE INTERNAL "")
include_directories(${VREP_DRIVER_INCLUDE_DIRS})
file(
    GLOB_RECURSE
    vrep_driver_lib_source_files
    ${CMAKE_CURRENT_SOURCE_DIR}/vrep_driver/*
)

add_library(vrep_driver SHARED ${vrep_driver_lib_source_files})
target_link_libraries(vrep_driver vrep_remote_api)
set_property(TARGET vrep_driver PROPERTY CXX_STANDARD 14)
