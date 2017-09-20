/*      File: main.cpp
*       This file is part of the program open-phri
*       Program description : OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
*       Copyright (C) 2017 -  Benjamin Navarro (LIRMM). All Right reserved.
*
*       This software is free software: you can redistribute it and/or modify
*       it under the terms of the LGPL license as published by
*       the Free Software Foundation, either version 3
*       of the License, or (at your option) any later version.
*       This software is distributed in the hope that it will be useful,
*       but WITHOUT ANY WARRANTY without even the implied warranty of
*       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
*       LGPL License for more details.
*
*       You should have received a copy of the GNU Lesser General Public License version 3 and the
*       General Public License version 3 along with this program.
*       If not, see <http://www.gnu.org/licenses/>.
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
		ControlLevel::Joint,
		SAMPLE_TIME);

	driver.startSimulation();
	driver.enableSynchonous(true);
	while(not driver.getSimulationData()) {
		driver.nextStep();
	}

	/***			Controller configuration			***/
	auto safety_controller = SafetyController(robot);

	// Pose, twist and acceleration at the waypoints
	auto quat_1 = Eigen::Quaterniond::Identity();
	auto quat_2 = quat_1.integrate(Vector3d(0., 0., 0.25));
	auto quat_3 = quat_1.integrate(Vector3d(0., 0., -0.25));
	auto pose_1 = TrajectoryPoint<Pose>(
		Pose(Vector3d(-0.197, 0., 1.1249), quat_1),
		Twist(),
		Acceleration());

	auto pose_2 = TrajectoryPoint<Pose>(
		Pose(Vector3d(-0.25, 0., 1.), quat_2),
		Twist(),
		Acceleration());

	auto pose_3 = TrajectoryPoint<Pose>(
		Pose(Vector3d(-0.15, 0., 0.85), quat_3),
		Twist(),
		Acceleration());

	auto trajectory_generator = TaskSpaceTrajectoryGenerator(
		pose_1,
		SAMPLE_TIME,
		TrajectorySynchronization::SynchronizeWaypoints);

	safety_controller.add(
		"traj vel",
		VelocityProxy(
			trajectory_generator.getTwistOutput(),
			ReferenceFrame::Base));

	// Generate paths between waypoints with maximum twist and acceleration
	trajectory_generator.addPathTo(
		pose_2,
		Twist(Vector3d(0.05, 1., 0.05), 0.5*Vector3d::Ones()),
		Acceleration(Vector3d(0.1, 1., 0.1), 0.5*Vector3d::Ones()));

	trajectory_generator.addPathTo(
		pose_3,
		Twist(Vector3d(0.10, 1., 0.05), 0.5*Vector3d::Ones()),
		Acceleration(Vector3d(0.2, 1., 0.2), 0.5*Vector3d::Ones()));

	trajectory_generator.addPathTo(
		pose_1,
		Twist(Vector3d(0.05, 1., 0.05), 0.5*Vector3d::Ones()),
		Acceleration(Vector3d(0.1, 1., 0.1), 0.5*Vector3d::Ones()));

	try {
		trajectory_generator.computeTimings();
	}
	catch(std::exception& err) {
		driver.enableSynchonous(false);
		driver.stopSimulation();
		throw;
	}

	Clock clock(SAMPLE_TIME);
	DataLogger logger(
		"/tmp",
		clock.getTime(),
		true);

	logger.logExternalData("traj6d-pos", trajectory_generator.getPoseOutput()->translation().data(), 3);
	logger.logExternalData("traj6d-quat", trajectory_generator.getPoseOutput()->orientation().coeffs().data(), 4);
	logger.logExternalData("traj6d-vel", trajectory_generator.getTwistOutput()->data(), 6);
	logger.logSafetyControllerData(&safety_controller);
	logger.logRobotData(robot);

	driver.readJointPosition(robot->jointTargetPosition());

	signal(SIGINT, sigint_handler);
	bool end = false;
	while(not (_stop or end)) {
		if(driver.getSimulationData()) {
			end = trajectory_generator();
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
