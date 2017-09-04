CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)

option(GEN_PYTHON_BINDINGS 	"Generate bindings for Python" OFF)
option(RUN_PYTHON_TESTS 	"Run also the Python version of the tests" OFF)
option(FORCE_CMAKE_BUILD 	"Build with CMake only even if inside a PID workspace" OFF)
option(USE_OPENMP 			"Make use of OpenMP to parallize certain parts of the library" OFF)

list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

PROJECT(open-phri)

declare_PID_Package(
			AUTHOR 		Benjamin Navarro
			INSTITUTION	LIRMM
			MAIL		navarro@lirmm.fr
			ADDRESS 	git@github.com:BenjaminNavarro/open-phri.git
			YEAR 		2017
			LICENSE 	GNULGPL
			DESCRIPTION "OpenPHRI: a generic framework to easily and safely control robots in interactions with humans"
)

set_PID_Package_Version(0 7 0)

check_PID_Platform(CONFIGURATION posix)

#now finding packages
find_package (api-driver-vrep 0.7 REQUIRED)
declare_PID_Package_Dependency (PACKAGE api-driver-vrep NATIVE VERSION 0.7)

find_package (eigen-extensions 0.5.0 REQUIRED)
declare_PID_Package_Dependency(PACKAGE eigen-extensions NATIVE VERSION 0.5.0)

if(GEN_PYTHON_BINDINGS)
	find_package (boost 1.63.0 REQUIRED)
	declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.63.0)

	find_package(PythonLibs REQUIRED)
endif()

if(RUN_PYTHON_TESTS)
    find_package(PythonInterp REQUIRED)
endif()

build_PID_Package()
