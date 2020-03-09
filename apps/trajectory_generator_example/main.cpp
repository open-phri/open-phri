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

bool _stop = false;

TaskSpaceTrajectoryGenerator createTrajectoryGenerator(double sample_time);

int main(int argc, char const* argv[]) {

    AppMaker app("configuration_examples/kuka_lwr4.yaml");

    /***			Create and configure the trajectory generator			***/
    auto trajectory_generator =
        createTrajectoryGenerator(app.getDriver()->getSampleTime());
    try {
        trajectory_generator.computeTimings();
    } catch (std::exception& err) {
        auto& driver = dynamic_cast<VREPDriver&>(*app.getDriver());
        driver.enableSynchonous(false);
        driver.stop();
        throw err;
    }

    /***			Controller configuration			***/
    app.getController()->add(
        "traj vel", VelocityProxy(trajectory_generator.getTwistOutput(),
                                  ReferenceFrame::Base));

    /**			Additional logs			***/
    auto logger = app.getDataLogger();
    logger->logExternalData(
        "traj6d-pos",
        trajectory_generator.getPoseOutput()->translation().data(), 3);
    logger->logExternalData(
        "traj6d-quat",
        trajectory_generator.getPoseOutput()->orientation().coeffs().data(), 4);
    logger->logExternalData("traj6d-vel",
                            trajectory_generator.getTwistOutput()->data(), 6);

    /***		Application initialization		***/
    bool init_ok = app.init();

    if (init_ok)
        std::cout << "Starting main loop\n";
    else
        std::cout << "Initialization failed\n";

    signal(SIGINT, [](int) { _stop = true; });

    /***		Main loop		***/
    while (init_ok and not _stop) {
        _stop |= not app.run([&trajectory_generator]() {
            return not trajectory_generator();
        } // call the trajectory generator before the controller
        );
    }

    signal(SIGINT, nullptr);

    /***		Application deinitialization		***/
    app.stop();

    return 0;
}

TaskSpaceTrajectoryGenerator createTrajectoryGenerator(double sample_time) {

    // Pose, twist and acceleration at the waypoints
    auto quat_1 = Eigen::Quaterniond::Identity();
    auto quat_2 = quat_1.integrate(Vector3d(0., 0., 0.25));
    auto quat_3 = quat_1.integrate(Vector3d(0., 0., -0.25));
    auto pose_1 =
        TrajectoryPoint<Pose>(Pose(Vector3d(-0.197, 0., 1.1249), quat_1),
                              spatial::Velocity(), Acceleration());

    auto pose_2 = TrajectoryPoint<Pose>(Pose(Vector3d(-0.25, 0., 1.), quat_2),
                                        spatial::Velocity(), Acceleration());

    auto pose_3 = TrajectoryPoint<Pose>(Pose(Vector3d(-0.15, 0., 0.85), quat_3),
                                        spatial::Velocity(), Acceleration());

    auto trajectory_generator = TaskSpaceTrajectoryGenerator(
        pose_1, sample_time, TrajectorySynchronization::SynchronizeWaypoints);

    // Generate paths between waypoints with maximum twist and acceleration
    trajectory_generator.addPathTo(
        pose_2,
        spatial::Velocity(Vector3d(0.05, 1., 0.05), 0.5 * Vector3d::Ones()),
        Acceleration(Vector3d(0.1, 1., 0.1), 0.5 * Vector3d::Ones()));

    trajectory_generator.addPathTo(
        pose_3,
        spatial::Velocity(Vector3d(0.10, 1., 0.05), 0.5 * Vector3d::Ones()),
        Acceleration(Vector3d(0.2, 1., 0.2), 0.5 * Vector3d::Ones()));

    trajectory_generator.addPathTo(
        pose_1,
        spatial::Velocity(Vector3d(0.05, 1., 0.05), 0.5 * Vector3d::Ones()),
        Acceleration(Vector3d(0.1, 1., 0.1), 0.5 * Vector3d::Ones()));

    return trajectory_generator;
}
