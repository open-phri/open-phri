from minieigen import *
from openphri import *

from signal import *

SAMPLE_TIME = 0.01
stop = False

def sigint_handler(signal, frame):
    global stop
    stop = True

###                 Controller configuration                ###
damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 500)
ext_force = NewVector6dPtr()
activation_force_threshold = NewDoublePtr(25)
deactivation_force_threshold = NewDoublePtr(5)

safety_controller = NewSafetyController(damping_matrix)
tcp_velocity = safety_controller.getTCPVelocity()

stop_constraint = NewEmergencyStopConstraint(ext_force, activation_force_threshold, deactivation_force_threshold)

constant_vel = NewVector6dPtr()
constant_velocity_generator = NewVelocityProxy(constant_vel)
constant_force_generator = NewForceProxy(ext_force)

safety_controller.addConstraint("stop constraint", stop_constraint)
safety_controller.addVelocityGenerator("vel proxy", constant_velocity_generator)
safety_controller.addForceGenerator("force proxy", constant_force_generator)

###				         V-REP driver	       		  ###
driver = NewVREPDriver(
    SAMPLE_TIME,
    "LBR4p_")      # Robot prefix

driver.enableSynchonous(True)
driver.startSimulation()

signal(SIGINT, sigint_handler)

t = 0.
while not stop:
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
