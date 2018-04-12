CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
include(Package_Definition NO_POLICY_SCOPE)

option(GEN_PYTHON_BINDINGS 	"Generate bindings for Python" OFF)
option(RUN_PYTHON_TESTS 	"Run also the Python version of the tests" OFF)
option(USE_OPENMP 			"Make use of OpenMP to parallize certain parts of the library" OFF)

PROJECT(open-phri)

declare_PID_Package(
			AUTHOR 		    Benjamin Navarro
			INSTITUTION	    LIRMM
			MAIL		    navarro@lirmm.fr
			ADDRESS 	    git@github.com:BenjaminNavarro/open-phri.git
            PUBLIC_ADDRESS  https://github.com/BenjaminNavarro/open-phri.git
			YEAR 		    2017
			LICENSE 	    GNULGPL
			DESCRIPTION     "OpenPHRI: a generic framework to easily and safely control robots in interactions with humans"
			README			instructions.md
)

set_PID_Package_Version(0 8 0)

check_PID_Platform(CONFIGURATION posix)

#now finding packages
declare_PID_Package_Dependency (PACKAGE api-driver-vrep NATIVE VERSION 1.0)

declare_PID_Package_Dependency(PACKAGE eigen-extensions NATIVE VERSION 0.6.0)

if(GEN_PYTHON_BINDINGS)
	declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.63.0 VERSION 1.64.0)

	find_package(PythonLibs REQUIRED)
endif()

if(RUN_PYTHON_TESTS)
    find_package(PythonInterp REQUIRED)
endif()

build_PID_Package()
