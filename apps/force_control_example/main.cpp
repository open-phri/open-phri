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

    auto maximum_velocity = make_shared<double>(0.1);
    auto velocity_constraint =
        make_shared<VelocityConstraint>(maximum_velocity);

    auto target_force = make_shared<Vector6d>(Vector6d::Zero());
    auto p_gain = make_shared<Vector6d>(Vector6d::Ones() * 0.001);
    auto d_gain = make_shared<Vector6d>(Vector6d::Ones() * 0.0003);
    auto selection = make_shared<Vector6d>(Vector6d::Zero());

    target_force->z() = 10.;
    selection->z() = 1.;

    auto force_control = make_shared<ForceControl>(target_force, SAMPLE_TIME,
                                                   p_gain, d_gain, selection);

    safety_controller.add("velocity constraint", velocity_constraint);

    safety_controller.add("force control", force_control);

    signal(SIGINT, sigint_handler);

    while (not driver.read() and not _stop) {
        usleep(SAMPLE_TIME * 1e6);
    }
    driver.enableSynchonous(true);

    if (not _stop)
        std::cout << "Starting main loop\n";
    while (not _stop) {
        if (driver.read()) {
            safety_controller.compute();
            if (not driver.send()) {
                std::cerr << "Can'send robot data to V-REP" << std::endl;
            }
        } else {
            std::cerr << "Can't get robot data from V-REP" << std::endl;
        }

        // std::cout <<
        // "**********************************************************************\n";
        // std::cout << "vel    : " << robot->jointVelocity()->transpose() <<
        // "\n"; std::cout << "pos msr: " <<
        // robot->jointCurrentPosition()->transpose() << "\n"; std::cout << "pos
        // tgt: " << robot->jointTargetPosition()->transpose() << "\n";

        // usleep(SAMPLE_TIME*1e6);
        driver.nextStep();
    }

    driver.enableSynchonous(false);
    driver.stop();

    return 0;
}
