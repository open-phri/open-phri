from minieigen import *
from pyrscl import *

from signal import *

SAMPLE_TIME = 0.01
stop = False

def sigint_handler(signal, frame):
    global stop
    stop = True

###                 Controller configuration                ###
damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 100)
safety_controller = NewSafetyController(damping_matrix)

tcp_velocity = safety_controller.getTCPVelocity()
total_velocity = safety_controller.getTotalVelocity()

maximum_velocity = NewDoublePtr(0.1)
velocity_constraint = NewVelocityConstraint(
    total_velocity,
    maximum_velocity)

reference_vel = NewVector6dPtr()
constant_vel_gen = NewVelocityProxy(reference_vel)

ext_force = NewVector6dPtr()
ext_force_generator = NewForceProxy(ext_force)

safety_controller.addConstraint(
    "velocity constraint",
    velocity_constraint)

safety_controller.addVelocityGenerator(
    "vel proxy",
    constant_vel_gen)

safety_controller.addForceGenerator(
    "ext force proxy",
    ext_force_generator)

###				         V-REP driver	       		  ###
driver = NewVREPDriver(
    SAMPLE_TIME,
    "LBR4p_")      # Robot prefix

driver.enableSynchonous(True)
driver.startSimulation()

signal(SIGINT, sigint_handler)

while not stop:
    driver.readTCPWrench(ext_force)
    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)

    driver.nextStep()

driver.enableSynchonous(False)
driver.stopSimulation()
