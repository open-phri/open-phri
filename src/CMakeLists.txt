###         OpenPHRI        ###
if(USE_OPENMP)
    declare_PID_Component(
        SHARED_LIB
        NAME open-phri
        DIRECTORY OpenPHRI
        CXX_STANDARD 14
        EXPORTED COMPILER_OPTIONS -fopenmp
        RUNTIME_RESOURCES robot_models
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
        CXX_STANDARD 14
        RUNTIME_RESOURCES robot_models
    )
endif()

declare_PID_Component_Dependency(
    COMPONENT open-phri
    EXPORT NATIVE eigen-extensions
    PACKAGE eigen-extensions
)

declare_PID_Component_Dependency(
    COMPONENT open-phri
    EXPORT NATIVE rpathlib
    PACKAGE pid-rpath
)

declare_PID_Component_Dependency(
    COMPONENT open-phri
    NATIVE rbdyn
    PACKAGE rbdyn
)

declare_PID_Component_Dependency(
    COMPONENT open-phri
    NATIVE rbdyn-parsers
    PACKAGE rbdyn
)

declare_PID_Component_Dependency(
    COMPONENT open-phri
    EXPORT EXTERNAL libyaml
    PACKAGE yaml-cpp
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
    declare_PID_Component(
        MODULE_LIB
        NAME openphri
        DIRECTORY pyOpenPHRI
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        NATIVE open-phri
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        NATIVE open-phri-vrep-driver
    )

    declare_PID_Component_Dependency(
  		COMPONENT openphri
  		EXPORT EXTERNAL boost-python
        PACKAGE boost
    )

    declare_PID_Component_Dependency(
        COMPONENT openphri
        EXPORT LINKS SHARED ${PYTHON_LIBRARIES}# ${Boost_LIBRARIES}
        INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}# ${Boost_INCLUDE_DIRS}
    )

    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES PREFIX "") # python needs a openphri.so file (without lib prefix)
    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES OUTPUT_NAME "openphri") # python needs a openphri.so file (without -dbg suffix)
endif()
