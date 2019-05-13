from minieigen import *
from openphri import *

from signal import *
import math

SAMPLE_TIME = 0.01
stop = False



def sigint_handler(signal, frame):
    global stop

    stop = True


###				         V-REP driver	       		  ###
driver = NewVREPDriver(
    SAMPLE_TIME,
    "LBR4p_")      # Robot prefix

driver.start()

###                 Controller configuration                ###
damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 100)
safety_controller = NewSafetyController(damping_matrix)

tcp_velocity = safety_controller.getTCPVelocity()

max_vel_interpolator = NewLinearInterpolator(
    NewLinearPoint(0.1, 0.),    # 0m/s at 0.1m
    NewLinearPoint(0.5, 0.2))   # 0.2m/s at 0.5m

max_vel_interpolator.enableSaturation(True)

maximum_velocity = max_vel_interpolator.getOutput()
velocity_constraint = NewVelocityConstraint(maximum_velocity)

    velocity_constraint,
    max_vel_interpolator)
    velocity_constraint,
    max_vel_interpolator)

separation_dist_vel_cstr.setVerbose(True)
obs1 = driver.trackObjectPosition("obstacle1", ReferenceFrame.TCP)
obs2 = driver.trackObjectPosition("obstacle2", ReferenceFrame.TCP)
separation_dist_vel_cstr.add("obstacle1", obs1)
ext_force = Newstd: : shared_ptr < Vector6d > ()

ext_force = Newstd: : shared_ptr < Vector6d > ()
ext_force_generator = NewForceProxy(ext_force)

safety_controller.addConstraint(
    "velocity constraint",
    separation_dist_vel_cstr)
    "ext force proxy",
    ext_force_generator)
    "ext force proxy",
    ext_force_generator)

driver.enableSynchonous(True)

signal(SIGINT, sigint_handler)

while not stop:
    driver.readTCPWrench(ext_force)
    driver.updateTrackedObjectsPosition()
    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)

    driver.nextStep()

driver.enableSynchonous(False)
driver.stop()
