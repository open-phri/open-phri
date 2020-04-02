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

phri::TaskSpaceTrajectoryGenerator
createTrajectoryGenerator(const phri::AppMaker& app);

int main() {
    // Create an application using a configuration file
    phri::AppMaker app{"trajectory_generator_example/app_config.yaml"};

    // Configure the trajectory generator
    auto trajectory_generator = createTrajectoryGenerator(app);

    // Generate the trajectory with the given targets and settings
    trajectory_generator.computeTimings();

    // Configure the controller
    app.controller().add<phri::VelocityProxy>(
        "traj vel", trajectory_generator.getTwistOutput());

    // Initialize the application. Exit on failure.
    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    // Update lambda function to be passed to the app in the control loop
    auto update_trajectory = [&]() {
        return not trajectory_generator.compute();
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

phri::TaskSpaceTrajectoryGenerator
createTrajectoryGenerator(const phri::AppMaker& app) {

    const auto frame = app.robot().controlPointParentFrame();

    auto delta_rotation = spatial::AngularPosition::Zero(frame);
    delta_rotation.orientation() = Eigen::Quaterniond::fromAngles(0., 0., 0.25);

    auto pose1 = spatial::Position::Zero(frame);
    pose1.linear() << -0.197, 0., 1.1249;

    auto pose2 = pose1;
    pose2.linear() << -0.25, 0., 1.;
    pose2.angular() += delta_rotation;

    auto pose3 = pose1;
    pose3.linear() << -0.15, 0., 0.85;
    pose3.angular() -= delta_rotation;

    auto zero_vel = spatial::Velocity::Zero(frame);
    auto zero_acc = spatial::Acceleration::Zero(frame);

    // Pose, twist and acceleration at the waypoints
    auto waypoint1 =
        phri::TaskSpaceTrajectoryGenerator::Point(pose1, zero_vel, zero_acc);
    auto waypoint2 =
        phri::TaskSpaceTrajectoryGenerator::Point(pose2, zero_vel, zero_acc);
    auto waypoint3 =
        phri::TaskSpaceTrajectoryGenerator::Point(pose3, zero_vel, zero_acc);

    auto trajectory_generator = phri::TaskSpaceTrajectoryGenerator(
        waypoint1, app.robot().control().timeStep(),
        phri::TrajectorySynchronization::SynchronizeWaypoints);

    // Generate paths between waypoints with maximum twist and acceleration
    auto max_vel = spatial::Velocity{frame};
    max_vel.angular().setConstant(0.5);

    auto max_acc = spatial::Acceleration{frame};
    max_acc.angular().setConstant(0.5);

    max_vel.linear() << 0.05, 1., 0.05;
    max_acc.linear() << 0.1, 1., 0.1;
    trajectory_generator.addPathTo(waypoint2, max_vel, max_acc);

    max_vel.linear() << 0.10, 1., 0.05;
    max_acc.linear() << 0.2, 1., 0.2;
    trajectory_generator.addPathTo(waypoint3, max_vel, max_acc);

    max_vel.linear() << 0.05, 1., 0.05;
    max_acc.linear() << 0.1, 1., 0.1;
    trajectory_generator.addPathTo(waypoint1, max_vel, max_acc);

    return trajectory_generator;
}
