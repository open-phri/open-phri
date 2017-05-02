#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <RSCL/RSCL.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace RSCL;
using namespace vrep;

constexpr double SAMPLE_TIME = 0.010;

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {

	/***				V-REP driver				***/
	VREPDriver driver(
		SAMPLE_TIME,
		"LBR4p_");      // LBR4p_: Robot prefix

	driver.startSimulation();

	/***			Controller configuration			***/
	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 100.);
	auto safety_controller = SafetyController(damping_matrix);

	auto tcp_velocity = safety_controller.getTCPVelocity();
	auto total_velocity = safety_controller.getTotalVelocity();

	auto max_vel_interpolator = make_shared<LinearInterpolator>(
		make_shared<LinearPoint>(0.1, 0.),     // 0m/s at 0.1m
		make_shared<LinearPoint>(0.5, 0.2));   // 0.2m/s at 0.5m

	max_vel_interpolator->enableSaturation(true);

	auto maximum_velocity = max_vel_interpolator->getOutput();
	auto velocity_constraint = make_shared< VelocityConstraint >(
		total_velocity,
		maximum_velocity);

	// Objects are tracked in the TCP frame so there is no need to provide the robot position
	auto separation_dist_vel_cstr = make_shared<SeparationDistanceConstraint>(
		velocity_constraint,
		max_vel_interpolator);

	separation_dist_vel_cstr->setVerbose(true);
	separation_dist_vel_cstr->add("obstacle1", driver.trackObjectPosition("obstacle1", ReferenceFrame::TCP));
	separation_dist_vel_cstr->add("obstacle2", driver.trackObjectPosition("obstacle2", ReferenceFrame::TCP));

	auto ext_force = make_shared<Vector6d>(Vector6d::Zero());
	auto ext_force_generator = make_shared<ForceProxy>(ext_force);

	safety_controller.addConstraint(
		"velocity constraint",
		separation_dist_vel_cstr);

	safety_controller.addForceGenerator(
		"ext force proxy",
		ext_force_generator);

	signal(SIGINT, sigint_handler);

	driver.enableSynchonous(true);

	cout << "Starting main loop" << endl;
	while(not _stop) {
		driver.readTCPWrench(ext_force);
		driver.updateTrackedObjectsPosition();
		safety_controller.updateTCPVelocity();
		driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame::TCP);

		driver.nextStep();
	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
