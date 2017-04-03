from minieigen import *
from PyRSCL import *

import sys
import math

damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 100.)
safety_controller = SafetyController(damping_matrix)
safety_controller.setVerbose(True)

tcp_velocity = safety_controller.getTCPVelocity()

rob_pos = NewVector6dPtr()
potential_field_generator = NewPotentialFieldGenerator(rob_pos)
potential_field_generator.setVerbose(True)

safety_controller.addForceGenerator("potential field", potential_field_generator)

obs_pos = NewVector6dPtr()
obstacle = NewPotentialFieldObject(
	PotentialFieldType.Repulsive,
	NewDoublePtr(10.),   # gain
	NewDoublePtr(0.2),   # threshold distance
	obs_pos)

tgt_pos = NewVector6dPtr()
target = NewPotentialFieldObject(
	PotentialFieldType.Attractive,
	NewDoublePtr(10.),  		# gain
	NewDoublePtr(math.inf),   	# threshold distance
	tgt_pos)

# Step #1 : addObject
ok = potential_field_generator.addObject("obstacle", obstacle)
assert_msg("Step #1", ok == True)

# Step #2 : re-addObject, force=False
ok = potential_field_generator.addObject("obstacle", obstacle)
assert_msg("Step #2", ok == False)

# Step #3 : re-addObject, force=True
ok = potential_field_generator.addObject("obstacle", obstacle, True)
assert_msg("Step #3", ok == True)

# Step #4 : removeObject
ok = potential_field_generator.removeObject("obstacle")
assert_msg("Step #4", ok == True)

# Step #5 : re-removeObject
ok = potential_field_generator.removeObject("obstacle")
assert_msg("Step #5", ok == False)

# Step #6 : getObject
potential_field_generator.addObject("obstacle", obstacle)
obj = potential_field_generator.getObject("obstacle")
assert_msg("Step #6", obj == obstacle)

# Step #7 : 1 obstacle > threshold distance
obs_pos[0] = 0.3
safety_controller.updateTCPVelocity()
assert_msg("Step #7", tcp_velocity.isApprox(Vector6.Zero))

# Step #8 : 1 obstacle < threshold distance
obs_pos[0] = 0.1
safety_controller.updateTCPVelocity()
assert_msg("Step #8", tcp_velocity.dot(obs_pos) < 0.)

# Step #9 : 1 target
ok = potential_field_generator.removeObject("obstacle")
potential_field_generator.addObject("target", target)
tgt_pos[0] = 0.1
tgt_pos[1] = 0.2
safety_controller.updateTCPVelocity()
assert_msg("Step #9", tcp_velocity.dot(tgt_pos) > 0.)

sys.exit(0)
