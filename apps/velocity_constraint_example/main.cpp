/* 	File: main.cpp
*	This file is part of the program open-phri
*  	Program description : OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
*  	Copyright (C) 2017 -  Benjamin Navarro (LIRMM). All Right reserved.
*
*	This software is free software: you can redistribute it and/or modify
*	it under the terms of the LGPL license as published by
*	the Free Software Foundation, either version 3
*	of the License, or (at your option) any later version.
*	This software is distributed in the hope that it will be useful,
*	but WITHOUT ANY WARRANTY without even the implied warranty of
*	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*	LGPL License for more details.
*
*	You should have received a copy of the GNU Lesser General Public License version 3 and the
*	General Public License version 3 along with this program.
*	If not, see <http://www.gnu.org/licenses/>.
*/
#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <OpenPHRI/OpenPHRI.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace phri;
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
