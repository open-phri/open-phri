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

    auto max_vel_interpolator = make_shared<LinearInterpolator>(
        make_shared<LinearPoint>(0.1, 0.),   // 0m/s at 0.1m
        make_shared<LinearPoint>(0.5, 0.2)); // 0.2m/s at 0.5m

    max_vel_interpolator->enableSaturation(true);

    auto maximum_velocity = max_vel_interpolator->getOutput();
    auto velocity_constraint =
        make_shared<VelocityConstraint>(maximum_velocity);

    // Objects are tracked in the TCP frame so there is no need to provide the
    // robot position
    auto separation_dist_vel_cstr = make_shared<SeparationDistanceConstraint>(
        velocity_constraint, max_vel_interpolator);

    separation_dist_vel_cstr->setVerbose(true);
    separation_dist_vel_cstr->add(
        "obstacle1",
        driver.trackObjectPosition("obstacle1", ReferenceFrame::TCP));
    separation_dist_vel_cstr->add(
        "obstacle2",
        driver.trackObjectPosition("obstacle2", ReferenceFrame::TCP));

    auto ext_force_generator =
        make_shared<ForceProxy>(robot->controlPointExternalForce());

    safety_controller.addConstraint("velocity constraint",
                                    separation_dist_vel_cstr);

    safety_controller.addForceGenerator("ext force proxy", ext_force_generator);

    signal(SIGINT, sigint_handler);

    usleep(10. * SAMPLE_TIME * 1e6);

    cout << "Starting main loop" << endl;
    while (not _stop) {
        if (driver.read()) {
            safety_controller.compute();
            driver.send();
        }

        usleep(SAMPLE_TIME * 1e6);
    }

    driver.stop();

    return 0;
}
