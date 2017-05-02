#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <RSCL/RSCL.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace RSCL;
using namespace vrep;

constexpr double SAMPLE_TIME = 0.010;

// 1 for position control, 0 for velocity control
#define POSITION_OUTPUT 0

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {
	/***				V-REP driver				***/
	VREPDriver driver(
		SAMPLE_TIME,
		"LBR4p_");      // Robot prefix


	/***			Controller configuration			***/
	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 500.);
	auto safety_controller = SafetyController(damping_matrix);
	auto tcp_velocity = safety_controller.getTCPVelocity();

	auto x_point_1 = make_shared<TrajectoryPoint>(-0.197,  0.,     0.);
	auto x_point_2 = make_shared<TrajectoryPoint>(-0.25,    0.,     0.);
	auto x_point_3 = make_shared<TrajectoryPoint>(-0.15,    0.,     0.);

	auto z_point_1 = make_shared<TrajectoryPoint>(1.1249,  0.,     0.);
	auto z_point_2 = make_shared<TrajectoryPoint>(1.,      -0.025, 0.);
	auto z_point_3 = make_shared<TrajectoryPoint>(0.85,    0.,     0.);


#if POSITION_OUTPUT
	auto robot_position = make_shared<Vector6d>();
	auto target_position = make_shared<Vector6d>();
	auto stiffness = make_shared<StiffnessGenerator>(
		make_shared<Matrix6d>(Matrix6d::Identity() * 5000.),
		target_position,
		robot_position);

	safety_controller.add("stiffness", stiffness);

	auto x_traj = make_shared<Trajectory>(TrajectoryOutputType::Position, x_point_1, &(target_position->x()), SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory>(TrajectoryOutputType::Position, z_point_1, &(target_position->z()), SAMPLE_TIME);
#else
	auto traj_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto traj_vel_gen = make_shared<VelocityProxy>(traj_vel);

	safety_controller.addVelocityGenerator(
		"traj vel",
		traj_vel_gen);

	auto x_traj = make_shared<Trajectory>(TrajectoryOutputType::Velocity, x_point_1, &(traj_vel->x()), SAMPLE_TIME);
	auto z_traj = make_shared<Trajectory>(TrajectoryOutputType::Velocity, z_point_1, &(traj_vel->z()), SAMPLE_TIME);
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

	auto trajectory_generator = TrajectoryGenerator(TrajectorySynchronization::SynchronizeWaypoints);
	trajectory_generator.add("x_traj", x_traj);
	trajectory_generator.add("z_traj", z_traj);

	trajectory_generator.computeParameters();

	driver.startSimulation();
	driver.enableSynchonous(true);
	signal(SIGINT, sigint_handler);

#if POSITION_OUTPUT
	driver.readTCPPose(target_position, ReferenceFrame::Base);
#endif

	bool end = false;
	while(not (_stop or end)) {
#if POSITION_OUTPUT
		driver.readTCPPose(robot_position, ReferenceFrame::Base);
#endif

		end = trajectory_generator.compute();

		safety_controller.updateTCPVelocity();
		driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame::TCP);

		driver.nextStep();
	}

	cout << (end ? "End of the trajectory reached" : "Trajectory generation interrupted") << endl;

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
