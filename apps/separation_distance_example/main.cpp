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

#include <physical_quantities/units/units.h>
#include <pid/signal_manager.h>

#include <iostream>

int main(int argc, char const* argv[]) {
    using namespace units::literals;
    // Create an application using a configuration file
    phri::AppMaker app{"separation_distance_example/app_config.yaml"};

    // Set the task space damping matrix
    app.robot().control().task().damping().diagonal().setConstant(100.);

    auto& driver = dynamic_cast<phri::VREPDriver&>(app.driver());

    // Create an interpolator providing the maximum velocity for a given
    // distance
    using Interpolator =
        phri::LinearInterpolator<scalar::Position, scalar::Velocity>;
    auto max_vel_interpolator = Interpolator{
        {scalar::Position{10_cm}, scalar::Velocity{0_m / units::second()}},
        {scalar::Position{50_cm}, scalar::Velocity{20_cm / units::second()}}};

    max_vel_interpolator.enableSaturation(true);

    // Configure the controller

    // Objects are tracked in the TCP frame so there is no need to
    // provide the robot position
    auto separation_distance =
        app.controller()
            .add<phri::SeparationDistanceConstraint<phri::VelocityConstraint,
                                                    Interpolator>>(
                "adpative vmax",
                phri::VelocityConstraint{max_vel_interpolator.output()},
                std::move(max_vel_interpolator));

    separation_distance->setVerbose(true);
    separation_distance->add("obstacle1",
                             driver.trackObjectPosition(
                                 "obstacle1", app.robot().controlPointFrame()));
    separation_distance->add("obstacle2",
                             driver.trackObjectPosition(
                                 "obstacle2", app.robot().controlPointFrame()));

    app.controller().add<phri::ExternalForce>("external force");

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
