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

#include <pid/signal_manager.h>

#include <iostream>

int main() {
    // Create an application using a configuration file
    phri::AppMaker app{"velocity_constraint_example/app_config.yaml"};

    // Set the task space damping matrix
    app.robot().control().task().damping().diagonal().setConstant(100.);

    // Configure the controller
    scalar::Velocity vmax{0.1};
    app.controller().add<phri::VelocityConstraint>("vmax", vmax);
    app.controller().add<phri::ExternalForce>("ext force");

    // Initialize the application. Exit on failure.
    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    // Catch CTRL-C signal
    bool stop = false;
    pid::SignalManager::registerCallback(pid::SignalManager::Interrupt, "stop",
                                         [&stop](int) { stop = true; });
    // Run the main loop
    while (not stop) {
        if (not app()) {
            // Communication error
            break;
        }
    }

    // Stop catching CTRL-C
    pid::SignalManager::unregisterCallback(pid::SignalManager::Interrupt,
                                           "stop");
}
