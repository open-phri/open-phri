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
// #include <OpenPHRI/drivers/vrep_driver.h>

#include <pid/signal_manager.h>
#include <yaml-cpp/yaml.h>

#include <iostream>

class VirtualForce : public phri::Driver {
public:
    VirtualForce(phri::Robot& robot)
        : phri::Driver{robot, robot.control().timeStep()} {
    }

    virtual bool start(double timeout = 30.) override {
        taskState().force().setZero();
        return true;
    }

    virtual bool stop() override {
        return true;
    }

    virtual bool read() override {
        taskState().force().x() += 0.05;
        return true;
    }

    virtual bool send() override {
        return true;
    }
};

int main() {
    // Create an application using a configuration file
    phri::AppMaker app("configuration_examples/kuka_lwr4.yaml");

    // Set the task space damping matrix
    app.robot().control().task().damping().diagonal().setConstant(100.);

    std::cout << "Controlled frame: " << app.robot().controlPointFrame()
              << std::endl;

    auto virtual_force = VirtualForce{app.robot()};

    // Configure the controller
    app.controller().add<phri::ExternalForce>("ext force");
    app.controller().add<phri::VelocityConstraint>("vmax", 0.1);
    app.controller().add<phri::VelocityProxy>(
        "vref", spatial::Velocity::Ones(app.robot().controlPointParentFrame()));

    // Initialize the application. Exit on failure.
    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    virtual_force.init();

    // Catch CTRL-C signal
    bool stop = false;
    pid::SignalManager::registerCallback(pid::SignalManager::Interrupt, "stop",
                                         [&stop](int) { stop = true; });
    // Run the main loop
    while (not stop) {
        virtual_force.read();
        if (not app()) {
            // Communication error
            break;
        }
    }

    std::cout << app.robot().task().command().velocity().frame() << std::endl;
    std::cout << app.robot().task().state().force().frame() << std::endl;

    // Stop catching CTRL-C
    pid::SignalManager::unregisterCallback(pid::SignalManager::Interrupt,
                                           "stop");
}
