#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <OpenPHRI/OpenPHRI.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace OpenPHRI;
using namespace vrep;

constexpr double SAMPLE_TIME = 0.010;

// 1 for position control, 0 for velocity control
#define POSITION_OUTPUT 0

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {

	/***				Robot				***/
	auto robot = make_shared<Robot>(
		"LBR4p",    // Robot's name, must match V-REP model's name
		7);         // Robot's joint count

	/***				V-REP driver				***/
	VREPDriver driver(
		robot,
		ControlLevel::TCP,
		SAMPLE_TIME);

	driver.startSimulation();

	/***			Controller configuration			***/
	*robot->controlPointDampingMatrix() *= 100.;
	auto safety_controller = SafetyController(robot);

	auto ext_force = robot->controlPointExternalForce();
	auto activation_force_threshold = make_shared<double>(25.);
	auto deactivation_force_threshold = make_shared<double>(5.);

	auto stop_constraint = make_shared<StopConstraint>(activation_force_threshold, deactivation_force_threshold);
	safety_controller.addConstraint("stop constraint", stop_constraint);

	auto external_force_generator = make_shared<ForceProxy>(ext_force);
	safety_controller.addForceGenerator("external force", external_force_generator);

	auto x_point_1 = make_shared<TrajectoryPoint<double>>(-0.197,  0.,     0.);
	auto x_point_2 = make_shared<TrajectoryPoint<double>>(-0.25,    0.,     0.);
	auto x_point_3 = make_shared<TrajectoryPoint<double>>(-0.15,    0.,     0.);

	auto z_point_1 = make_shared<TrajectoryPoint<double>>(1.1249,  0.,     0.);
	auto z_point_2 = make_shared<TrajectoryPoint<double>>(1.,      -0.025, 0.);
	auto z_point_3 = make_shared<TrajectoryPoint<double>>(0.85,    0.,     0.);

	auto robot_position = robot->controlPointCurrentPose();
	auto target_position = robot->controlPointTargetPose();

#if POSITION_OUTPUT
	auto stiffness = make_shared<StiffnessGenerator>(
		make_shared<Matrix6d>(Matrix6d::Identity() * 5000.),
		target_position,
		robot_position);

	safety_controller.add("stiffness", stiffness);

	auto x_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Position, x_point_1, &(target_position->x()), SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Position, z_point_1, &(target_position->z()), SAMPLE_TIME);
#else
	auto traj_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto traj_vel_gen = make_shared<VelocityProxy>(traj_vel);

	safety_controller.addVelocityGenerator(
		"traj vel",
		traj_vel_gen);

	auto x_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Velocity, x_point_1, &(traj_vel->x()), SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory<double>>(TrajectoryOutputType::Velocity, z_point_1, &(traj_vel->z()), SAMPLE_TIME);
#endif


	x_traj->addPathTo(x_point_2, 0.05, 0.1);
	x_traj->addPathTo(x_point_3, 0.1, 0.2);
	x_traj->addPathTo(x_point_1, 0.05, 0.1);

	z_traj->addPathTo(z_point_2, 0.05, 0.1);
	z_traj->addPathTo(z_point_3, 0.05, 0.2);
	z_traj->addPathTo(z_point_1, 0.05, 0.1);

	// Use these for fixed-time paths
	// z_traj->addPathTo(z_point_2, 5.);
	// z_traj->addPathTo(z_point_3, 5.);
	// z_traj->addPathTo(z_point_1, 10.);

	auto trajectory_generator = PathFollower(TrajectorySynchronization::SynchronizeWaypoints, PathStopAction::StopOne);
	trajectory_generator.add("x_traj", x_traj, &(robot_position->x()), &(target_position->x()), 0.01);
	trajectory_generator.add("z_traj", z_traj, &(robot_position->z()), &(target_position->z()), 0.01);

	trajectory_generator.computeParameters();

	signal(SIGINT, sigint_handler);

	usleep(10.*SAMPLE_TIME*1e6);

#if POSITION_OUTPUT
	driver.readTCPPose(target_position, ReferenceFrame::Base);
#endif

	bool end = false;
	while(not (_stop or end)) {
		if(driver.getSimulationData()) {
			end = trajectory_generator.compute();
			safety_controller.compute();
			driver.sendSimulationData();
		}

		usleep(SAMPLE_TIME*1e6);
	}

	cout << (end ? "End of the trajectory reached" : "Trajectory generation interrupted") << endl;

	driver.stopSimulation();

	return 0;
}
