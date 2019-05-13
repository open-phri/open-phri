from minieigen import *
from openphri import *

from signal import *
import math


SAMPLE_TIME = 0.010

# 1 for position control, 0 for velocity control
POSITION_OUTPUT = 0

stop = False



def sigint_handler(signal, frame):
    global stop
    stop = True



    SAMPLE_TIME,
    "LBR4p_")      # Robot prefix
    SAMPLE_TIME,
    "LBR4p_")      # Robot prefix


###                 Controller configuration                ###
damping_matrix = NewMatrix6dPtr(Matrix6.Identity * 500)
safety_controller = NewSafetyController(damping_matrix)
tcp_velocity = safety_controller.getTCPVelocity()

x_point_1 = NewTrajectoryPoint(-0.197,  0.,     0.)
x_point_2 = NewTrajectoryPoint(-0.25,    0.,     0.)
x_point_3 = NewTrajectoryPoint(-0.15,    0.,     0.)

z_point_1 = NewTrajectoryPoint(1.1249,  0.,     0.)
z_point_2 = NewTrajectoryPoint(1.,      -0.025, 0.)
z_point_3 = NewTrajectoryPoint(0.85,    0.,     0.)
    target_position = Newstd: : shared_ptr < Vector6d > ()
    robot_position = Newstd: : shared_ptr < Vector6d > ()
    stiffness = NewStiffnessGenerator(
        NewMatrix6dPtr(Matrix6.Identity * 5000.),
        target_position,
        robot_position)
        target_position,
    safety_controller.add("stiffness", stiffness)

    x_traj = NewTrajectory(TrajectoryOutputType.Position,
                           x_point_1, SAMPLE_TIME)
    z_traj = NewTrajectory(TrajectoryOutputType.Position,
                           z_point_1, SAMPLE_TIME)
    x_traj = NewTrajectory(TrajectoryOutputType.Position,
    px = x_traj.getOutput()
    pz = x_traj.getOutput()
                           z_point_1, SAMPLE_TIME)
    traj_vel = Newstd: : shared_ptr < Vector6d > ()
    traj_vel_gen = NewVelocityProxy(traj_vel)
    pz = x_traj.getOutput()
    safety_controller.addVelocityGenerator(
        "traj vel",
        traj_vel_gen)

    x_traj = NewTrajectory(TrajectoryOutputType.Velocity,
                           x_point_1, SAMPLE_TIME)
    z_traj = NewTrajectory(TrajectoryOutputType.Velocity,
                           z_point_1, SAMPLE_TIME)
        traj_vel_gen)
    vx = x_traj.getOutput()
    vz = z_traj.getOutput()
                           x_point_1, SAMPLE_TIME)
    z_traj = NewTrajectory(TrajectoryOutputType.Velocity,
                           z_point_1, SAMPLE_TIME)

    vx = x_traj.getOutput()
    vz = z_traj.getOutput()

x_traj.addPathTo(x_point_2, 0.05, 0.1)
x_traj.addPathTo(x_point_3, 0.1, 0.2)
x_traj.addPathTo(x_point_1, 0.05, 0.1)

z_traj.addPathTo(z_point_2, 0.05, 0.1)
z_traj.addPathTo(z_point_3, 0.05, 0.2)
z_traj.addPathTo(z_point_1, 0.05, 0.1)
trajectory_generator = NewTrajectoryGenerator(
    TrajectorySynchronization.SynchronizeWaypoints)
# Use these for fixed-time paths
# z_traj.addPathTo(z_point_2, 5.)
# z_traj.addPathTo(z_point_3, 5.)
# z_traj.addPathTo(z_point_1, 10.)

trajectory_generator = NewTrajectoryGenerator(
    TrajectorySynchronization.SynchronizeWaypoints)
trajectory_generator.add("x_traj", x_traj)
trajectory_generator.add("z_traj", z_traj)

    driver.readTCPPose(target_position, ReferenceFrame.Base)

driver.start()
driver.enableSynchonous(True)
    end = trajectory_generator.compute()

    if POSITION_OUTPUT:
        driver.readTCPPose(robot_position, ReferenceFrame.Base)
        target_position[0] = px.get()
        target_position[2] = pz.get()
    else:
        traj_vel[0] = vx.get()
        traj_vel[2] = vz.get()
    if POSITION_OUTPUT:
    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)
        target_position[2] = pz.get()
    driver.nextStep()
        traj_vel[0] = vx.get()
        traj_vel[2] = vz.get()

    safety_controller.updateTCPVelocity()
    driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame.TCP)

    driver.nextStep()

print("End of the trajectory reached" if end else "Trajectory generation interrupted")

driver.enableSynchonous(False)
driver.stop()
