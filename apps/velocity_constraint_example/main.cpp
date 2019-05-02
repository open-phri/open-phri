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
#include <pid/rpath.h>
#include <yaml-cpp/yaml.h>

using namespace std;
using namespace phri;

bool _stop = false;

void sigint_handler(int sig) {
    _stop = true;
}

int main(int argc, char const* argv[]) {
    PID_EXE(argv[0]);

    AppMaker app("configuration_examples/kuka_lwr4.yaml");

    /***			Controller configuration			***/
    auto robot = app.getRobot();
    *robot->controlPointDampingMatrix() *= 100.;

    auto maximum_velocity = make_shared<double>(0.1);

    auto safety_controller = app.getController();
    safety_controller->add("velocity constraint",
                           VelocityConstraint(maximum_velocity));

    auto vel_cstr =
        safety_controller->get<VelocityConstraint>("velocity constraint");

    safety_controller->add("ext force proxy", ExternalForce(robot));

    bool init_ok = app.init();

    if (init_ok)
        std::cout << "Starting main loop\n";
    else
        std::cout << "Initialization failed\n";

    signal(SIGINT, sigint_handler);

    while (init_ok and not _stop) {
        _stop |= not app.run();
    }

    app.stop();

    return 0;
}
