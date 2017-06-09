#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <chrono>
#include <list>

#include <RSCL/RSCL.h>
#include <vrep_driver/vrep_driver.h>

#include "state_machine.h"

using namespace RSCL;
using namespace vrep;

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {

	/***				Robot				***/
	auto robot = std::make_shared<Robot>(
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
	auto laser_data = driver.initLaserScanner("Hokuyo");

	/***			Controller configuration			***/
	*robot->controlPointDampingMatrix() *= 500.;
	auto safety_controller = SafetyController(robot);

	auto laser_detector = LaserScannerDetector(
		laser_data,
		270. * M_PI / 180.,
		0.2,
		3.);

	auto state_machine = StateMachine(
		robot,
		safety_controller,
		laser_detector,
		argc > 1); // skip teaching

	signal(SIGINT, sigint_handler);

	while(not (driver.getSimulationData(ReferenceFrame::Base, ReferenceFrame::Base) and laser_data->size() == 1080)and not _stop) {
		usleep(SAMPLE_TIME*1e6);
	}
	driver.enableSynchonous(true);

	laser_detector.init();
	state_machine.init();

	if(not _stop)
		std::cout << "Starting main loop\n";

	while(not _stop) {
		if(driver.getSimulationData(ReferenceFrame::Base, ReferenceFrame::Base)) {

			_stop &= not state_machine.compute();

			safety_controller.compute();
			if(not driver.sendSimulationData()) {
				std::cerr << "Can'send robot data to V-REP" << std::endl;
			}
		}
		else {
			std::cerr << "Can't get robot data from V-REP" << std::endl;
		}

		driver.nextStep();
	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
