###         OpenPHRI        ###
if(USE_OPENMP)
    declare_PID_Component(
        SHARED_LIB
        NAME open-phri
        DIRECTORY OpenPHRI
        EXPORTED COMPILER_OPTIONS -std=c++14 -fopenmp
    )

    declare_PID_Component_Dependency(
        COMPONENT open-phri
        EXPORT LINKS SHARED -fopenmp
    )
else()
    declare_PID_Component(
        SHARED_LIB
        NAME open-phri
        DIRECTORY OpenPHRI
        EXPORTED COMPILER_OPTIONS -std=c++14
    )
endif()

declare_PID_Component_Dependency(
    COMPONENT open-phri
    EXPORT NATIVE eigen-extensions
    PACKAGE eigen-extensions
)


###         OpenPHRI V-REP DRIVER        ###
declare_PID_Component(
    SHARED_LIB
    NAME open-phri-vrep-driver
    DIRECTORY vrep_driver
)

declare_PID_Component_Dependency(
    COMPONENT open-phri-vrep-driver
    EXPORT NATIVE open-phri
)

declare_PID_Component_Dependency(
    COMPONENT open-phri-vrep-driver
    EXPORT NATIVE vrep-driver
    PACKAGE api-driver-vrep
)

###         pyOpenPHRI        ###
if(GEN_PYTHON_BINDINGS)
    # find_package(Boost COMPONENTS python3 REQUIRED)

    declare_PID_Component(
        MODULE_LIB
        NAME openphri
        DIRECTORY pyOpenPHRI
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        EXPORT NATIVE open-phri
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        EXPORT NATIVE open-phri-vrep-driver
    )

    declare_PID_Component_Dependency(
  		COMPONENT openphri
  		EXPORT EXTERNAL boost
  		INCLUDE_DIRS <boost>/include
        LINKS SHARED <boost>/lib/libboost_python3.so.1.63.0
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        EXPORT LINKS SHARED ${PYTHON_LIBRARIES}# ${Boost_LIBRARIES}
        INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}# ${Boost_INCLUDE_DIRS}
    )

    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES PREFIX "") # python needs a openphri.so file (without lib prefix)
    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES OUTPUT_NAME "openphri") # python needs a openphri.so file (without -dbg suffix)
endif()
