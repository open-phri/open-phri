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
		SAMPLE_TIME,
		"","",-1000);

	driver.startSimulation();

	/***			Controller configuration			***/
	auto safety_controller = SafetyController(robot);

	auto point_1 = TrajectoryPoint<Vector2d>(Vector2d(-0.197,   1.1249),    Vector2d(0., 0.),       Vector2d(0., 0.));
	auto point_2 = TrajectoryPoint<Vector2d>(Vector2d(-0.25,    1.),        Vector2d(0., -0.025),   Vector2d(0., 0.));
	auto point_3 = TrajectoryPoint<Vector2d>(Vector2d(-0.15,    0.85),      Vector2d(0., 0.),       Vector2d(0., 0.));

	auto target = make_shared<Vector6d>(Vector6d::Zero());

	auto trajectory_generator = TrajectoryGenerator<Vector2d>(point_1, SAMPLE_TIME, TrajectorySynchronization::SynchronizeWaypoints);

	*robot->controlPointDampingMatrix() *= 250.;
	auto robot_position = robot->controlPointCurrentPose();

	auto velocity_target = make_shared<Vector6d>();
	safety_controller.add(
		"traj vel",
		VelocityProxy(velocity_target));

	trajectory_generator.addPathTo(point_2, Vector2d(0.05, 0.05), Vector2d(0.1, 0.1));
	trajectory_generator.addPathTo(point_3, Vector2d(0.1, 0.05), Vector2d(0.2, 0.2));
	trajectory_generator.addPathTo(point_1, Vector2d(0.05, 0.05), Vector2d(0.1, 0.1));

	trajectory_generator.computeTimings();

	Clock clock(SAMPLE_TIME);
	DataLogger logger(
		"/tmp",
		clock.getTime(),
		true
		);

	logger.logExternalData("traj2d-pos", trajectory_generator.getPositionOutput()->data(), 2);
	logger.logExternalData("traj2d-vel", trajectory_generator.getVelocityOutput()->data(), 2);

	signal(SIGINT, sigint_handler);

	driver.enableSynchonous(true);
	driver.nextStep();
	driver.readTCPPose(target, ReferenceFrame::Base);

	bool end = false;
	while(not (_stop or end)) {
		if(driver.getSimulationData()) {
			end = trajectory_generator();

			velocity_target->x() = (*trajectory_generator.getVelocityOutput())[0];
			velocity_target->z() = (*trajectory_generator.getVelocityOutput())[1];

			safety_controller();
			driver.sendSimulationData();

			clock();
			logger();
		}

		driver.nextStep();
	}

	cout << (end ? "End of the trajectory reached" : "Trajectory generation interrupted") << endl;

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
