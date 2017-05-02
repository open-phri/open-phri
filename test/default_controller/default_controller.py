from minieigen import *
from PyRSCL import *
from math import *
import sys


def setIdentity(mat):
	for i in range(0,6):
		mat[i,i] = 1

damping_matrix = NewMatrix6dPtr()
setIdentity(damping_matrix)

safety_controller = NewSafetyController(damping_matrix)
safety_controller.setVerbose(True)
tcp_velocity = safety_controller.getTCPVelocity()

safety_controller.updateTCPVelocity()

assert_msg("Step #1", tcp_velocity.isApprox(Vector6.Zero))

sys.exit(0)
