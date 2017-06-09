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
	*robot->controlPointDampingMatrix() *= 500.;
	auto safety_controller = SafetyController(robot);

	auto ext_force = robot->controlPointExternalForce();
	auto activation_force_threshold = make_shared<double>(25.);
	auto deactivation_force_threshold = make_shared<double>(5.);

	auto stop_constraint = make_shared<StopConstraint>(activation_force_threshold, deactivation_force_threshold);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);
	auto external_force_generator = make_shared<ForceProxy>(ext_force);

	safety_controller.addConstraint("stop constraint", stop_constraint);
	safety_controller.addVelocityGenerator("vel proxy", constant_velocity_generator);
	safety_controller.addForceGenerator("force proxy", external_force_generator);

	signal(SIGINT, sigint_handler);

	usleep(10.*SAMPLE_TIME*1e6);

	double t = 0.;
	while(not _stop) {
		if(driver.getSimulationData()) {
			safety_controller.compute();
			driver.sendSimulationData();
		}

		if(t < 5.) {
			(*constant_vel)(0) = 0.05;
		}
		else if(t < 10.) {
			(*constant_vel)(0) = -0.05;
		}
		else {
			t = 0.;
		}

		t += SAMPLE_TIME;

		usleep(SAMPLE_TIME*1e6);
	}

	driver.stopSimulation();

	return 0;
}
