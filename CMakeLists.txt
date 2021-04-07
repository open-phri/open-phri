CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

option(GEN_PYTHON_BINDINGS 	"Generate bindings for Python" OFF)
option(RUN_PYTHON_TESTS 	"Run also the Python version of the tests" OFF)
option(USE_OPENMP 			"Make use of OpenMP to parallize certain parts of the library" OFF)

PROJECT(open-phri)

PID_Package(
	AUTHOR 		       Benjamin Navarro
	INSTITUTION	       LIRMM
	MAIL		       navarro@lirmm.fr
	ADDRESS 	       git@gite.lirmm.fr:rpc/control/open-phri.git
	PUBLIC_ADDRESS     https://gite.lirmm.fr/rpc/control/open-phri.git
	CONTRIBUTION_SPACE pid
	YEAR 		       2018-2021
	LICENSE 	       GNULGPL
	DESCRIPTION        "OpenPHRI: a generic framework to easily and safely control robots in interactions with humans"
	README			   instructions.md
	CODE_STYLE		   pid11
	VERSION			   2.0.0
)

# check_PID_Platform(CONFIGURATION posix) TODO OpenMP

PID_Dependency(api-driver-vrep 		VERSION 2.0)
PID_Dependency(eigen-extensions 	VERSION 0.6)
PID_Dependency(pid-rpath 			VERSION 2.0)
PID_Dependency(rbdyn 				FROM VERSION 1.4.1)
PID_Dependency(yaml-cpp 			FROM VERSION 0.6.2)
PID_Dependency(physical-quantities	EXACT VERSION 0.2.0)

if(BUILD_EXAMPLES)
	PID_Dependency(pid-os-utilities)
endif()

if(BUILD_AND_RUN_TESTS)
	PID_Dependency(catch2)
endif()

if(GEN_PYTHON_BINDINGS)
	PID_Dependency(boost)

	find_package(PythonLibs REQUIRED)
endif()

if(RUN_PYTHON_TESTS)
    find_package(PythonInterp REQUIRED)
endif()

PID_Publishing(
    PROJECT https://gite.lirmm.fr/rpc/control/open-phri
    FRAMEWORK rpc
    CATEGORIES algorithm/control 
    DESCRIPTION OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
    PUBLISH_BINARIES
    ALLOWED_PLATFORMS x86_64_linux_stdc++11
)

build_PID_Package()
