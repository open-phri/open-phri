function(add_application app)
  PID_Component(
    EXAMPLE_APPLICATION
    NAME ${app}
    DIRECTORY ${app}
    RUNTIME_RESOURCES ${app}
    DEPEND 
      open-phri/open-phri
      open-phri/open-phri-vrep-driver
      pid-os-utilities/pid-signal-manager
  )
endfunction()

# add_application(controller_benchmark)
# add_application(demo)
add_application(force_control_example)
add_application(joint_trajectory_generator_example)
# add_application(null_space_motion_example)
add_application(obstacle_avoidance_example)
# add_application(path_follower_example)
# add_application(separation_distance_example)
add_application(stop_constraint_example)
# add_application(trajectory_generator_benchmark)
add_application(trajectory_generator_example)
add_application(velocity_constraint_example)
