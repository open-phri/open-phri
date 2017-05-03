from minieigen import *
from pyrscl import *
import sys


def setIdentity(mat):
	for i in range(0,6):
		mat[i,i] = 1

damping_matrix = NewMatrix6dPtr()
setIdentity(damping_matrix)

safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)

maximum_velocity = NewDoublePtr()
maximum_velocity.set(0.1)
velocity_constraint = NewVelocityConstraint(maximum_velocity)

ok = safety_controller.addConstraint("velocity limit", velocity_constraint)
assert_msg("Step #1", ok == True)

cstr = safety_controller.getConstraint("velocity limit")
assert_msg("Step #2", cstr == velocity_constraint)

ok = safety_controller.addConstraint("velocity limit", velocity_constraint)
assert_msg("Step #3", ok == False)

ok = safety_controller.addConstraint("velocity limit", velocity_constraint, force=True)
assert_msg("Step #4", ok == True)

ok = safety_controller.removeConstraint("velocity limit")
assert_msg("Step #5", ok == True)

ok = safety_controller.removeConstraint("velocity limit")
assert_msg("Step #6", ok == False)

cstr = safety_controller.getConstraint("velocity limit")
assert_msg("Step #7", cstr == None)

sys.exit(0)
