###         OpenPHRI        ###
if(USE_OPENMP)
    set(OPENMP_COMPONENT_OPTIONS "EXPORTED COMPILER_OPTIONS -fopenmp LINKS -fopenmp")
endif()

PID_Component(
    SHARED_LIB
    NAME open-phri
    DIRECTORY OpenPHRI
    CXX_STANDARD 17
    EXPORT 
        eigen-extensions/eigen-extensions 
        pid-rpath/rpathlib
        yaml-cpp/libyaml
        physical-quantities/physical-quantities
    DEPEND 
        rbdyn/rbdyn 
        rbdyn/rbdyn-parsers
    RUNTIME_RESOURCES 
        robot_models
    ${OPENMP_COMPONENT_OPTIONS}
)

###         OpenPHRI V-REP DRIVER        ###
# PID_Component(
#     SHARED_LIB
#     NAME vrep-driver
#     DIRECTORY vrep_driver
#     EXPORT open-phri/open-phri api-driver-vrep/vrep-driver
# )

###         pyOpenPHRI        ###
if(GEN_PYTHON_BINDINGS)
    PID_Component(
        MODULE_LIB
        NAME openphri
        DIRECTORY pyOpenPHRI
        DEPEND open-phri/open-phri open-phri/vrep-driver boost/boost-python
    )

    PID_Component_Dependency(
        COMPONENT openphri
        EXPORT LINKS SHARED ${PYTHON_LIBRARIES}
        INCLUDE_DIRS ${PYTHON_INCLUDE_DIRS}
    )

    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES PREFIX "") # python needs a openphri.so file (without lib prefix)
    set_target_properties(openphri${INSTALL_NAME_SUFFIX} PROPERTIES OUTPUT_NAME "openphri") # python needs a openphri.so file (without -dbg suffix)
endif()
