#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <OpenPHRI/OpenPHRI.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace OpenPHRI;
using namespace vrep;

constexpr double SAMPLE_TIME = 0.010;

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

	auto max_vel_interpolator = make_shared<LinearInterpolator>(
		make_shared<LinearPoint>(0.1, 0.),     // 0m/s at 0.1m
		make_shared<LinearPoint>(0.5, 0.2));   // 0.2m/s at 0.5m

	max_vel_interpolator->enableSaturation(true);

	auto maximum_velocity = max_vel_interpolator->getOutput();
	auto velocity_constraint = make_shared<VelocityConstraint>(maximum_velocity);

	// Objects are tracked in the TCP frame so there is no need to provide the robot position
	auto separation_dist_vel_cstr = make_shared<SeparationDistanceConstraint>(
		velocity_constraint,
		max_vel_interpolator);

	separation_dist_vel_cstr->setVerbose(true);
	separation_dist_vel_cstr->add("obstacle1", driver.trackObjectPosition("obstacle1", ReferenceFrame::TCP));
	separation_dist_vel_cstr->add("obstacle2", driver.trackObjectPosition("obstacle2", ReferenceFrame::TCP));

	auto ext_force_generator = make_shared<ForceProxy>(robot->controlPointExternalForce());

	safety_controller.addConstraint(
		"velocity constraint",
		separation_dist_vel_cstr);

	safety_controller.addForceGenerator(
		"ext force proxy",
		ext_force_generator);

	signal(SIGINT, sigint_handler);

	usleep(10.*SAMPLE_TIME*1e6);

	cout << "Starting main loop" << endl;
	while(not _stop) {
		if(driver.getSimulationData()) {
			safety_controller.compute();
			driver.sendSimulationData();
		}

		usleep(SAMPLE_TIME*1e6);
	}

	driver.stopSimulation();

	return 0;
}
