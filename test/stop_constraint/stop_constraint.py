from minieigen import *
from PyRSCL import *
from math import *
import sys


damping_matrix = NewMatrix6dPtr(Matrix6.Identity)
safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)

tcp_velocity = safety_controller.getTCPVelocity()
total_velocity = safety_controller.getTotalVelocity()

ext_force = NewVector6dPtr()
activation_force_threshold = NewDoublePtr(25.)
deactivation_force_threshold = NewDoublePtr(5.)
stop_constraint = NewStopConstraint(ext_force, activation_force_threshold, deactivation_force_threshold)

constant_vel = NewVector6dPtr()
constant_velocity_generator = NewVelocityProxy(constant_vel)
constant_force_generator = NewForceProxy(ext_force)

safety_controller.addConstraint("stop constraint", stop_constraint)
safety_controller.addVelocityGenerator("vel proxy", constant_velocity_generator)
safety_controller.addForceGenerator("force proxy", constant_force_generator)

# Step #1 : no velocity, no force
safety_controller.updateTCPVelocity()

assert_msg("Step #1", tcp_velocity.isApprox(Vector6.Zero))

# Step #2 : velocity, no force
constant_vel[0] = 0.2
safety_controller.updateTCPVelocity()

assert_msg("Step #2", tcp_velocity.isApprox(total_velocity))

# Step #3 : velocity, force < low
ext_force[0] = 3
safety_controller.updateTCPVelocity()

assert_msg("Step #3", tcp_velocity.isApprox(total_velocity))

# Step #4 : velocity, low < force < max
ext_force[0] = 15
safety_controller.updateTCPVelocity()

assert_msg("Step #4", tcp_velocity.isApprox(total_velocity))

# Step #5 : velocity, force > max
ext_force[0] = 30
safety_controller.updateTCPVelocity()

assert_msg("Step #5", tcp_velocity.isApprox(Vector6.Zero))

# Step #6 : velocity, low < force < max
ext_force[0] = 15
safety_controller.updateTCPVelocity()

assert_msg("Step #6", tcp_velocity.isApprox(Vector6.Zero))

# Step #7 : velocity, force < low
ext_force[0] = 4
safety_controller.updateTCPVelocity()

assert_msg("Step #7", tcp_velocity.isApprox(total_velocity))

sys.exit(0)
