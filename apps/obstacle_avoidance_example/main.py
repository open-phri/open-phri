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

maximum_velocity = Newstd::shared_ptr<double>(0.1)
velocity_constraint = NewVelocityConstraint(maximum_velocity)

# Objects are tracked in the TCP frame so there is no need to provide the robot position
potential_field_generator = NewPotentialFieldGenerator()
potential_field_generator.setVerbose(True)

obstacle1 = NewPotentialFieldObject(
    PotentialFieldType.Repulsive,
    Newstd::shared_ptr<double>(10.),   # gain
    Newstd::shared_ptr<double>(0.2),   # threshold distance
    driver.trackObjectPosition("obstacle1", ReferenceFrame.TCP))

obstacle2 = NewPotentialFieldObject(
    PotentialFieldType.Repulsive,
    Newstd::shared_ptr<double>(10.),   # gain
    Newstd::shared_ptr<double>(0.2),   # threshold distance
    driver.trackObjectPosition("obstacle2", ReferenceFrame.TCP))

target = NewPotentialFieldObject(
    PotentialFieldType.Attractive,
    Newstd::shared_ptr<double>(10.),      # gain
    Newstd::shared_ptr<double>(math.inf), # threshold distance
    driver.trackObjectPosition("target", ReferenceFrame.TCP))

potential_field_generator.add("obstacle1", obstacle1)
potential_field_generator.add("obstacle2", obstacle2)
potential_field_generator.add("target", target)

safety_controller.addConstraint(
    "velocity constraint",
    velocity_constraint)


safety_controller.addForceGenerator(
	"potential field",
	potential_field_generator)


driver.enableSynchonous(True)

signal(SIGINT, sigint_handler)

while not stop:
    driver.updateTrackedObjectsPosition()
    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)

    driver.nextStep()

driver.enableSynchonous(False)
driver.stop()
