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
#include <cstdlib>

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

    phri::TrajectoryPoint<phri::VectorXd> joint_start_point;
    phri::TrajectoryPoint<phri::VectorXd> joint_end_point;

    joint_start_point.resize(app.getRobot()->jointCount());
    joint_end_point.resize(app.getRobot()->jointCount());

    auto joint_trajectory_generator = phri::TrajectoryGenerator<phri::VectorXd>(
        joint_start_point, app.getDriver()->getSampleTime(),
        phri::TrajectorySynchronization::SynchronizeTrajectory);

    phri::VectorXd dqmax(app.getRobot()->jointCount()),
        d2qmax(app.getRobot()->jointCount());
    dqmax.setConstant(1.5);
    d2qmax.setConstant(0.5);
    joint_trajectory_generator.addPathTo(joint_end_point, dqmax, d2qmax);

    app.getController()->add(
        "joint traj vel", phri::JointVelocityProxy(
                              joint_trajectory_generator.getVelocityOutput()));

    bool init_ok = app.init();

    if (init_ok)
        std::cout << "Starting main loop\n";
    else
        std::cout << "Initialization failed\n";

    signal(SIGINT, sigint_handler);

    *joint_start_point.y = *app.getRobot()->jointCurrentPosition();
    for (size_t i = 0; i < joint_start_point.y->size(); ++i) {
        (*joint_end_point.y)[i] =
            (*joint_start_point.y)[i] + (0.5 * std::rand()) / RAND_MAX;
    }

    joint_trajectory_generator.computeTimings();

    while (init_ok and not _stop) {
        _stop |= not app.run([&joint_trajectory_generator]() {
            return not joint_trajectory_generator.compute();
        });
    }

    app.stop();

    return 0;
}
