from minieigen import *
from PyRSCL import *
from math import *
import sys

def isClose(v1, v2, eps = 1e-3):
	return abs(v1-v2) < eps

def setZero(vec):
	for i in range(0,6):
		vec[i] = 0

def getTransVel(vec):
	t = Vector3.Zero
	for i in range(0,3):
		t[i] = vec[i]
	return t

damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 10)
maximum_power = NewDoublePtr(10)
external_force = NewVector6dPtr(Vector6.Zero)

safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)
tcp_velocity = safety_controller.getTCPVelocity()
total_velocity = safety_controller.getTotalVelocity()

power_constraint = NewPowerConstraint(total_velocity, external_force, maximum_power)
power_constraint_test = NewPowerConstraint(tcp_velocity, external_force, maximum_power)
power = power_constraint_test.getPower()

constant_vel = NewVector6dPtr()
constant_velocity_generator = NewVelocityProxy(constant_vel)

safety_controller.addConstraint("power constraint", power_constraint)
safety_controller.addVelocityGenerator("const vel", constant_velocity_generator)

# Step #1 : no velocity
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #1", isClose(power.get(), 0.))

# Step #2 : velocity 1 axis, no force
constant_vel[0] = 0.2
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #2", isClose(power.get(), 0.))

# Step #3 : velocity 1 axis, force same axis with opposite sign < max
external_force[0] = -10.
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #3", isClose(power.get(), -2.))

# Step #4 : velocity 1 axis, force same axis with same sign < max
external_force[0] = 10.
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #4", isClose(power.get(), 2.))

# Step #5 : velocity 1 axis, force same axis with opposite sign > max
external_force[0] = -100.
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #5", isClose(power.get(), -maximum_power.get()))

# Step #6 : velocity 1 axis, force same axis with same sign > max
external_force[0] = 100.
safety_controller.updateTCPVelocity()
power_constraint_test.compute()

assert_msg("Step #6", tcp_velocity.isApprox(total_velocity))

sys.exit(0)
