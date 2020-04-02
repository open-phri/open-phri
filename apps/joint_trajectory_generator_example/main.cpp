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
#include <random>

int main() {
    // Create an application using a configuration file
    phri::AppMaker app{"joint_trajectory_generator_example/app_config.yaml"};

    // Set the task space damping matrix
    app.robot().control().task().damping().diagonal().setConstant(100.);
    const auto dofs = app.robot().jointCount();

    // Configure the trajectory generator
    phri::JointTrajectoryGenerator::Point joint_start_point{dofs};
    phri::JointTrajectoryGenerator::Point joint_end_point{dofs};

    auto joint_trajectory_generator = phri::JointTrajectoryGenerator{
        joint_start_point, app.robot().control().timeStep(),
        phri::TrajectorySynchronization::SynchronizeWaypoints};

    auto max_velocity = vector::dyn::Velocity::Constant(dofs, 0.5);
    auto max_acceleration = vector::dyn::Acceleration::Constant(dofs, 1.);

    // Go to waypoint with given maximum velocity and acceleration
    joint_trajectory_generator.addPathTo(joint_end_point, max_velocity,
                                         max_acceleration);

    // Go back in 2 seconds
    joint_trajectory_generator.addPathTo(joint_start_point,
                                         scalar::Duration{2.});

    // Configure the controller
    vector::dyn::Velocity reference_velocity{dofs};
    app.controller().add<phri::JointVelocityProxy>(
        "joint trajectory", joint_trajectory_generator.getVelocityOutput());

    // Initialize the application. Exit on failure.
    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    // Generate random target positions
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-0.5, 0.5);

    *joint_start_point.y = app.robot().joints().state().position();
    for (size_t i = 0; i < dofs; ++i) {
        (*joint_end_point.y)[i] = (*joint_start_point.y)[i] + dis(gen);
    }

    // Generate the trajectory with the given targets and settings
    joint_trajectory_generator.computeTimings();

    // Update lambda function to be passed to the app in the control loop
    auto update_trajectory = [&]() {
        return not joint_trajectory_generator.compute();
    };

    // Catch CTRL-C signal
    bool stop = false;
    pid::SignalManager::registerCallback(pid::SignalManager::Interrupt, "stop",
                                         [&stop](int) { stop = true; });
    // Run the main loop
    while (not stop) {
        if (not app(update_trajectory)) {
            // Communication error
            break;
        }
    }

    // Stop catching CTRL-C
    pid::SignalManager::unregisterCallback(pid::SignalManager::Interrupt,
                                           "stop");
}
