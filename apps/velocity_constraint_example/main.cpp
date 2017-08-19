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
		SAMPLE_TIME,
		"", "127.0.0.1", -1000
		);

	driver.startSimulation();

	/***			Controller configuration			***/
	*robot->controlPointDampingMatrix() *= 100.;
	auto safety_controller = SafetyController(robot);

	auto maximum_velocity = make_shared<double>(0.1);

	safety_controller.add(
		"velocity constraint",
		VelocityConstraint(maximum_velocity));

	safety_controller.add(
		"ext force proxy",
		ExternalForce(robot));

	signal(SIGINT, sigint_handler);

	driver.enableSynchonous(true);
	while(not driver.getSimulationData() and not _stop) {
		driver.nextStep();
	}

	if(not _stop)
		std::cout << "Starting main loop\n";
	while(not _stop) {
		if(driver.getSimulationData()) {
			safety_controller();
			if(not driver.sendSimulationData()) {
				std::cerr << "Can'send simulation data to V-REP" << std::endl;
			}
		}
		else {
			std::cerr << "Can't get simulation data from V-REP" << std::endl;
		}

		driver.nextStep();
	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
