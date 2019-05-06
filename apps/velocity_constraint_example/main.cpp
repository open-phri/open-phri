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

bool _stop = false;

void sigint_handler(int sig) {
    _stop = true;
}

int main(int argc, char const* argv[]) {
    phri::AppMaker app("configuration_examples/kuka_lwr4.yaml");

    auto& robot = *app.getRobot();
    robot.control.joints.damping.setConstant(100.);

    auto safety_controller = app.getController();
    safety_controller->add("vmax", phri::VelocityConstraint(0.1));
    safety_controller->add("ext force", phri::ExternalForce());

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
