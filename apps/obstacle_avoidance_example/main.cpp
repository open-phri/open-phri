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
    phri::AppMaker app{"obstacle_avoidance_example/app_config.yaml"};

    // Set the task space damping matrix
    app.robot().control().task().damping().diagonal().setConstant(100.);

    auto& driver = dynamic_cast<phri::VREPDriver&>(app.driver());

    // Configure the controller
    scalar::Velocity vmax{0.1};
    app.controller().add<phri::VelocityConstraint>("vmax", vmax);

    auto potential_field_generator =
        app.controller().add<phri::PotentialFieldGenerator>("potential field");
    potential_field_generator->setVerbose(true);

    auto obstacle1 = phri::PotentialFieldObject(
        phri::PotentialFieldType::Repulsive,
        10., // gain
        0.2, // threshold distance
        driver.trackObjectPosition("obstacle1",
                                   app.robot().controlPointFrame()));

    auto obstacle2 = phri::PotentialFieldObject(
        phri::PotentialFieldType::Repulsive,
        10., // gain
        0.2, // threshold distance
        driver.trackObjectPosition("obstacle2",
                                   app.robot().controlPointFrame()));

    auto target = phri::PotentialFieldObject(
        phri::PotentialFieldType::Attractive,
        10.,                                     // gain
        std::numeric_limits<double>::infinity(), // threshold distance
        driver.trackObjectPosition("target", app.robot().controlPointFrame()));

    potential_field_generator->add("obstacle1", obstacle1);
    potential_field_generator->add("obstacle2", obstacle2);
    potential_field_generator->add("target", target);

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
