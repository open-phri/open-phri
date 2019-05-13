from minieigen import *
from openphri import *

import sys
import math

damping_matrix = NewMatrix6dPtr(Matrix6.Identity)
safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)

tcp_velocity = safety_controller.getTCPVelocity()

rob_pos = Newstd::shared_ptr<Vector6d>()
potential_field_generator = NewPotentialFieldGenerator(rob_pos)
potential_field_generator.setVerbose(True)

safety_controller.addForceGenerator("potential field", potential_field_generator)

obs_pos = Newstd::shared_ptr<Vector6d>()
obstacle = NewPotentialFieldObject(
	PotentialFieldType.Repulsive,
	Newstd::shared_ptr<double>(10.),   # gain
	Newstd::shared_ptr<double>(0.2),   # threshold distance
	obs_pos)

tgt_pos = Newstd::shared_ptr<Vector6d>()
target = NewPotentialFieldObject(
	PotentialFieldType.Attractive,
	Newstd::shared_ptr<double>(10.),  		# gain
	Newstd::shared_ptr<double>(math.inf),   	# threshold distance
	tgt_pos)

# Step #1 : add
ok = potential_field_generator.add("obstacle", obstacle)
assert_msg("Step #1", ok == True)

# Step #2 : re-add, force=False
ok = potential_field_generator.add("obstacle", obstacle)
assert_msg("Step #2", ok == False)

# Step #3 : re-add, force=True
ok = potential_field_generator.add("obstacle", obstacle, True)
assert_msg("Step #3", ok == True)

# Step #4 : remove
ok = potential_field_generator.remove("obstacle")
assert_msg("Step #4", ok == True)

# Step #5 : re-remove
ok = potential_field_generator.remove("obstacle")
assert_msg("Step #5", ok == False)

# Step #6 : get
potential_field_generator.add("obstacle", obstacle)
obj = potential_field_generator.get("obstacle")
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
ok = potential_field_generator.remove("obstacle")
potential_field_generator.add("target", target)
tgt_pos[0] = 0.1
tgt_pos[1] = 0.2
safety_controller.updateTCPVelocity()
assert_msg("Step #9", tcp_velocity.dot(tgt_pos) > 0.)

sys.exit(0)
