CMAKE_MINIMUM_REQUIRED(VERSION 3.0.2)

option(GEN_PYTHON_BINDINGS 	"Generate bindings for Python" OFF)
option(RUN_PYTHON_TESTS 	"Run also the Python version of the tests" OFF)

# Check if we're inside a PID workspace, if not, fall back to a pure CMake configuration
set(WORKSPACE_DIR ${CMAKE_SOURCE_DIR}/../.. CACHE PATH "root of the packages workspace directory")
if(EXISTS ${WORKSPACE_DIR}/share/cmake/system)
	list(APPEND CMAKE_MODULE_PATH ${WORKSPACE_DIR}/share/cmake/system) # using generic scripts/modules of the workspace
	include(Package_Definition NO_POLICY_SCOPE)
	set(PID_WS_FOUND TRUE)

	PROJECT(rscl)

	declare_PID_Package(
				AUTHOR 		Benjamin Navarro
				INSTITUTION	LIRMM
				MAIL		navarro@lirmm.fr
				ADDRESS 	git@gite.lirmm.fr:navarro/RSCL.git
				YEAR 		2017
				LICENSE 	GNULGPL
				DESCRIPTION "obot Safety Control Library: provides a generic framework to easily control robots in operational space with safety constraints"
	)

	set_PID_Package_Version(0 1)

	check_PID_Platform(CONFIGURATION posix)

	#now finding packages
	find_package (api-driver-vrep 0.6 REQUIRED) #api-driver-vrep is required
	declare_PID_Package_Dependency (PACKAGE api-driver-vrep NATIVE VERSION 0.6)

	find_package (eigen 3.2.9 REQUIRED) #eigen is required
	declare_PID_Package_Dependency(	PACKAGE eigen EXTERNAL VERSION 3.2.9)

	if(GEN_PYTHON_BINDINGS)
		find_package (boost 1.63.0 REQUIRED) #boost is required
		declare_PID_Package_Dependency(PACKAGE boost EXTERNAL VERSION 1.63.0)

		find_package(PythonLibs REQUIRED)
	endif()

	if(RUN_PYTHON_TESTS)
	    find_package(PythonInterp REQUIRED)
	endif()

	build_PID_Package()
else()
	include(CMakeLists.cmake)
endif()
