/*      File: main.cpp
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2017 -
 * Benjamin Navarro (LIRMM). All Right reserved.
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
 *       You should have received a copy of the GNU Lesser General Public
 * License version 3 and the General Public License version 3 along with this
 * program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/vrep_driver.h>

using namespace std;
using namespace phri;

constexpr double SAMPLE_TIME = 0.010;

// 1 for position control, 0 for velocity control
#define POSITION_OUTPUT 0

bool _stop = false;

void sigint_handler(int sig) {
    _stop = true;
}

int main(int argc, char const* argv[]) {

    /***				Robot				***/
    auto robot = make_shared<Robot>(
        "LBR4p", // Robot's name, must match V-REP model's name
        7);      // Robot's joint count

    /***				V-REP driver				***/
    VREPDriver driver(robot, SAMPLE_TIME);

    driver.start();

    /***			Controller configuration			***/
    *robot->controlPointDampingMatrix() *= 100.;
    auto safety_controller = SafetyController(robot);

    auto ext_force = robot->controlPointExternalForce();
    auto activation_force_threshold = make_shared<double>(25.);
    auto deactivation_force_threshold = make_shared<double>(5.);

    safety_controller.add("stop constraint", EmergencyStopConstraint(
                                                 activation_force_threshold,
                                                 deactivation_force_threshold));
    safety_controller.add("external force", ExternalForce(robot));

    auto point_1 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.197, 1.1249), Vector2d(0., 0.), Vector2d(0., 0.));
    auto point_2 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.25, 1.), Vector2d(0., -0.025), Vector2d(0., 0.));
    auto point_3 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.15, 0.85), Vector2d(0., 0.), Vector2d(0., 0.));

    auto velocity_target = make_shared<Twist>();
    auto position_target = make_shared<Pose>();

    *robot->controlPointDampingMatrix() *= 250.;
    auto robot_position = robot->controlPointCurrentPose();

    safety_controller.add(
        "stiffness",
        StiffnessGenerator(make_shared<Matrix6d>(Matrix6d::Identity() * 5000.),
                           position_target, ReferenceFrame::TCP));

    safety_controller.add("traj vel", VelocityProxy(velocity_target));

    auto trajectory_generator = TrajectoryGenerator<Vector2d>(
        point_1, SAMPLE_TIME, TrajectorySynchronization::SynchronizeWaypoints);

    trajectory_generator.addPathTo(point_2, Vector2d(0.05, 0.05),
                                   Vector2d(0.1, 0.1));
    trajectory_generator.addPathTo(point_3, Vector2d(0.1, 0.05),
                                   Vector2d(0.2, 0.2));
    trajectory_generator.addPathTo(point_1, Vector2d(0.05, 0.05),
                                   Vector2d(0.1, 0.1));

    auto reference = make_shared<Vector2d>();
    trajectory_generator.enableErrorTracking(reference, Vector2d(0.01, 0.01),
                                             true);

    trajectory_generator.computeTimings();

    Clock clock(SAMPLE_TIME);
    DataLogger logger("/tmp", clock.getTime(), true);

    logger.logExternalData("traj6d-vel", velocity_target->translation().data(),
                           3);
    logger.logExternalData("traj6d-pos", position_target->translation().data(),
                           3);
    logger.logExternalData("rob-pos", robot_position->translation().data(), 3);

    signal(SIGINT, sigint_handler);

    driver.enableSynchonous(true);
    driver.nextStep();
    driver.readTCPPose(position_target, ReferenceFrame::Base);

    trajectory_generator();

    bool end = false;
    while (not(_stop or end)) {
        if (driver.read()) {
            (*reference)[0] = robot_position->translation().x();
            (*reference)[1] = robot_position->translation().z();

            end = trajectory_generator();

            position_target->translation().x() =
                (*trajectory_generator.getPositionOutput())[0];
            position_target->translation().z() =
                (*trajectory_generator.getPositionOutput())[1];
            velocity_target->translation().x() =
                (*trajectory_generator.getVelocityOutput())[0];
            velocity_target->translation().z() =
                (*trajectory_generator.getVelocityOutput())[1];

            safety_controller();
            driver.send();

            clock();
            logger();
        }
        driver.nextStep();
    }

    cout << (end ? "End of the trajectory reached"
                 : "Trajectory generation interrupted")
         << endl;

    driver.enableSynchonous(false);
    driver.stop();

    return 0;
}
