from minieigen import *
from pyrscl import *
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

damping_matrix = NewMatrix6dPtr(Matrix6.Identity)
maximum_velocity = NewDoublePtr(0.5)

safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)
tcp_velocity = safety_controller.getTCPVelocity()
total_velocity = safety_controller.getTotalVelocity()

velocity_constraint = NewVelocityConstraint(maximum_velocity)

constant_vel = NewVector6dPtr()
constant_velocity_generator = NewVelocityProxy(constant_vel)

safety_controller.addConstraint("velocity constraint", velocity_constraint)
safety_controller.addVelocityGenerator("vel proxy", constant_velocity_generator)

# Step #1 : no velocity
safety_controller.updateTCPVelocity()

assert_msg("Step #1", tcp_velocity.isApprox(Vector6.Zero))

# Step #2 : velocity 1 axis < max
constant_vel[0] = 0.2
safety_controller.updateTCPVelocity()

assert_msg("Step #2", tcp_velocity.isApprox(total_velocity))

# Step #3 : velocity 1 axis > max
constant_vel[0] = 0.6
safety_controller.updateTCPVelocity()

assert_msg("Step #3", isClose(getTransVel(tcp_velocity).norm(), maximum_velocity.get()))

# Step #4 : velocity 3 axes < max
constant_vel[0] = 0.2
constant_vel[1] = 0.1
constant_vel[2] = 0.3
safety_controller.updateTCPVelocity()

assert_msg("Step #4", tcp_velocity.isApprox(total_velocity))

# Step #5 : velocity 3 axes > max
constant_vel[0] = 0.5
constant_vel[1] = 0.4
constant_vel[2] = 0.6
safety_controller.updateTCPVelocity()

assert_msg("Step #5", isClose(getTransVel(tcp_velocity).norm(), maximum_velocity.get()))

# Step #6 : rotational velocity only
setZero(constant_vel)
constant_vel[4] = 0.5
constant_vel[4] = 0.4
constant_vel[5] = 0.6
safety_controller.updateTCPVelocity()

assert_msg("Step #6", tcp_velocity.isApprox(total_velocity))

sys.exit(0)
