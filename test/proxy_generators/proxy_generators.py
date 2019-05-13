from minieigen import *
from openphri import *
from math import *
import sys

sys.settrace


def isClose(v1, v2, eps=1e-3):
    return abs(v1-v2) < eps


def setZero(vec):
    for i in range(0, 6):
        vec[i] = 0


def setIdentity(mat):
    for i in range(0, 6):
        mat[i, i] = 1


damping_matrix = NewMatrix6dPtr()
setIdentity(damping_matrix)
damping_matrix *= 10

constant_vel = Newstd: : shared_ptr < Vector6d > ()
constant_force = Newstd: : shared_ptr < Vector6d > ()

constant_velocity_generator = NewVelocityProxy(constant_vel)
constant_force_generator = NewForceProxy(constant_force)

safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)
tcp_velocity = safety_controller.getTCPVelocity()

safety_controller.addVelocityGenerator(
    "vel proxy", constant_velocity_generator)
safety_controller.addForceGenerator("force proxy", constant_force_generator)

# Step #1 : no velocity, no force
safety_controller.updateTCPVelocity()

assert_msg("Step #1", tcp_velocity.isApprox(Vector6.Zero))

# Step #2 : velocity 1 axis, no force
constant_vel[0] = 0.5

safety_controller.updateTCPVelocity()

assert_msg("Step #2", isClose(tcp_velocity.norm(), 0.5))

# Step #3 : velocity 2 axes, no force
constant_vel[0] = 1
constant_vel[3] = 1

safety_controller.updateTCPVelocity()

assert_msg("Step #3", isClose(tcp_velocity.norm(), sqrt(2)))

# Step #4 : no velocity, force 1 axis
setZero(constant_vel)
constant_force[2] = 20

safety_controller.updateTCPVelocity()

assert_msg("Step #4", isClose(tcp_velocity.norm(), 2))

# Step #5 : no velocity, force 2 axes
constant_force[2] = 10
constant_force[5] = 10

safety_controller.updateTCPVelocity()

assert_msg("Step #5", isClose(tcp_velocity.norm(), sqrt(2)))

# Step #6 : velocity 3 axes, force 3 axes, separate axes
setZero(constant_vel)
setZero(constant_force)
constant_vel[0] = 1
constant_vel[2] = 1
constant_vel[4] = 1
constant_force[1] = 10
constant_force[3] = 10
constant_force[5] = 10

safety_controller.updateTCPVelocity()

assert_msg("Step #6", isClose(tcp_velocity.norm(), sqrt(6)))

# Step #7 : velocity 3 axes, force 3 axes, mixed axes
setZero(constant_vel)
setZero(constant_force)
constant_vel[0] = 1
constant_vel[3] = 1
constant_vel[4] = 1
constant_force[1] = 10
constant_force[3] = 10
constant_force[5] = 10

safety_controller.updateTCPVelocity()

assert_msg("Step #7", isClose(tcp_velocity.norm(), sqrt(8)))

sys.exit(0)
