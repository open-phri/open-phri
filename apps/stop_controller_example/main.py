from minieigen import *
from PyRSCL import *

import signal

SAMPLE_TIME = 0.005
stop = False

def sigint_handler(signal, frame):
    global stop
    stop = True

damping_matrix = Matrix6.Identity * 500

ext_force = Vector6.Zero
activation_force_threshold = NewDoublePtr()
activation_force_threshold.set(25)
deactivation_force_threshold = NewDoublePtr()
deactivation_force_threshold.set(5)

safety_controller = SafetyController(damping_matrix)
tcp_velocity = safety_controller.getTCPVelocity()

stop_constraint = NewStopConstraint(ext_force, activation_force_threshold, deactivation_force_threshold)

constant_vel = Vector6.Zero
constant_velocity_generator = NewConstantVelocityGenerator(constant_vel)
constant_force_generator = NewConstantForceGenerator(ext_force);

safety_controller.addConstraint("stop constraint", stop_constraint, False)
safety_controller.addVelocityGenerator("const vel", constant_velocity_generator, False)
safety_controller.addForceGenerator("const force", constant_force_generator, False)

driver = VREPDriver(
    SAMPLE_TIME,
    "LBR4p_",
    "",
    "127.0.0.1",
    19997)

driver.enableSynchonous(True)
driver.startSimulation()

signal.signal(signal.SIGINT, sigint_handler)

t = 0.
while stop == False:
    driver.readTCPWrench(ext_force)
    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)

    if t < 5:
    	constant_vel[0] = 0.05
    elif t < 10:
    	constant_vel[0] = -0.05
    else:
    	t = 0.

    t += SAMPLE_TIME

    driver.nextStep()

driver.stopSimulation()
