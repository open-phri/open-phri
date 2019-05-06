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

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/vrep_driver.h>

#include <pid/rpath.h>
#include <pid/signal_manager.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

int main(int argc, char const* argv[]) {
    phri::AppMaker app("configuration_examples/kuka_lwr4.yaml");

    app.robot().control.task.damping.setConstant(100.);

    app.controller().add("vmax", phri::VelocityConstraint(0.1));
    app.controller().add("ext force", phri::ExternalForce());

    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    bool stop = false;
    pid::SignalManager::registerCallback(pid::SignalManager::Interrupt, "stop",
                                         [&stop](int) { stop = true; });
    while (not stop) {
        stop |= not app.run();
    }

    pid::SignalManager::unregisterCallback(pid::SignalManager::Interrupt,
                                           "stop");
    app.stop();
}
