CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)

option(GEN_PYTHON_BINDINGS 	"Generate bindings for Python" OFF)
option(RUN_PYTHON_TESTS 	"Run also the Python version of the tests" OFF)

# Check if we're inside a PID workspace, if not, fall back to a pure CMake configuration
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
if(EXISTS ${WORKSPACE_DIR}/share/cmake/system)
	list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
	include(Package_Definition NO_POLICY_SCOPE)
	include(CMakeLists.pid)
else()
	include(CMakeLists.cmake)
endif()
