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
		SAMPLE_TIME,
		"", "127.0.0.1", -1000
		);

	driver.startSimulation();

	/***			Controller configuration			***/
	*robot->controlPointDampingMatrix() *= 100.;
	auto safety_controller = SafetyController(robot);

	auto maximum_velocity = make_shared<double>(0.1);
	auto velocity_constraint = make_shared<VelocityConstraint>(maximum_velocity);

	auto target_force = make_shared<Vector6d>(Vector6d::Zero());
	auto p_gain = make_shared<Vector6d>(Vector6d::Ones() * 0.001);
	auto d_gain = make_shared<Vector6d>(Vector6d::Ones() * 0.0003);
	auto selection = make_shared<Vector6d>(Vector6d::Zero());

	target_force->z() = 10.;
	selection->z() = 1.;

	auto force_control = make_shared<ForceControl>(
		robot->controlPointExternalForce(),
		target_force,
		SAMPLE_TIME,
		p_gain,
		d_gain,
		selection);

	safety_controller.add(
		"velocity constraint",
		velocity_constraint);

	safety_controller.add(
		"force control",
		force_control);

	signal(SIGINT, sigint_handler);

	while(not driver.getSimulationData() and not _stop) {
		usleep(SAMPLE_TIME*1e6);
	}
	driver.enableSynchonous(true);

	if(not _stop)
		std::cout << "Starting main loop\n";
	while(not _stop) {
		if(driver.getSimulationData()) {
			safety_controller.compute();
			if(not driver.sendSimulationData()) {
				std::cerr << "Can'send robot data to V-REP" << std::endl;
			}
		}
		else {
			std::cerr << "Can't get robot data from V-REP" << std::endl;
		}

		// std::cout << "**********************************************************************\n";
		// std::cout << "vel    : " << robot->jointVelocity()->transpose() << "\n";
		// std::cout << "pos msr: " << robot->jointCurrentPosition()->transpose() << "\n";
		// std::cout << "pos tgt: " << robot->jointTargetPosition()->transpose() << "\n";

		// usleep(SAMPLE_TIME*1e6);
		driver.nextStep();

	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
