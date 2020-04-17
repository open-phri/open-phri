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

#include <physical_quantities/spatial/impedance.h>

#include <pid/signal_manager.h>

#include <iostream>

phri::TaskSpaceTrajectoryGenerator
createTrajectoryGenerator(const phri::AppMaker& app);

int main() {
    // Create an application using a configuration file
    phri::AppMaker app{"path_follower_example/app_config.yaml"};

    // Set the task space damping matrix
    auto stiffness = spatial::Stiffness::Zero(app.robot().controlPointFrame());
    stiffness.linear().setConstant(5000);
    stiffness.angular().setConstant(500);

    auto damping = spatial::Damping::Zero(app.robot().controlPointFrame());
    damping.diagonal() =
        2. * stiffness.diagonal().cwiseSqrt(); // Critically damped

    app.robot().control().task().damping() = damping;

    // Configure the trajectory generator
    auto trajectory_generator = createTrajectoryGenerator(app);

    // Configure the controller
    scalar::Velocity vmax{0.1};
    // app.controller().add<phri::ExternalForce>("ext force");
    app.controller().add<phri::TaskEmergencyStopConstraint>(
        "Emergency stop", scalar::Force{25.}, scalar::Force{5.});
    app.controller().add<phri::VelocityProxy>(
        "traj vel", trajectory_generator.getTwistOutput());
    app.controller().add<phri::StiffnessGenerator>(
        "stiffness", stiffness, trajectory_generator.getPoseOutput());

    Eigen::Vector6d error_threshold;
    error_threshold.head<3>().setConstant(0.01);
    error_threshold.tail<3>().setConstant(0.1);
    double hysteresis = 0.1; // 10 percent
    trajectory_generator.enableErrorTracking(
        &app.robot().task().state().position(), error_threshold, true,
        hysteresis);

    // Initialize the application. Exit on failure.
    if (app.init()) {
        std::cout << "Starting main loop" << std::endl;
    } else {
        std::cerr << "Initialization failed" << std::endl;
        std::exit(-1);
    }

    // Generate the trajectory with the given targets and settings
    trajectory_generator.computeTimings();

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

/*

int main(int argc, char const* argv[]) {

    auto robot = make_shared<Robot>(
        "LBR4p", // Robot's name, must match V-REP model's name
        7);      // Robot's joint count

    VREPDriver driver(robot, SAMPLE_TIME);

    driver.start();

    auto point_1 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.197, 1.1249), Vector2d(0., 0.), Vector2d(0., 0.));
    auto point_2 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.25, 1.), Vector2d(0., -0.025), Vector2d(0., 0.));
    auto point_3 = TrajectoryPoint<Vector2d>(
        Vector2d(-0.15, 0.85), Vector2d(0., 0.), Vector2d(0., 0.));

    auto velocity_target = make_shared<spatial::Velocity>();
    auto position_target = make_shared<Pose>();

    *robot->controlPointDampingMatrix() *= 250.;
    auto robot_position = robot->controlPointCurrentPose();

    safety_controller.add(
        "stiffness",
        StiffnessGenerator(make_shared<Matrix6d>(Matrix6d::Identity() * 5000.),
                           position_target, ReferenceFrame::TCP));

    safety_controller.add("traj vel", VelocityProxy(velocity_target));

    auto trajectory_generator = TrajectoryGenerator<Vector2d>(
        point_1, SAMPLE_TIME, TrajectorySynchronization::SynchronizeWaypoints);

    trajectory_generator.addPathTo(point_2, Vector2d(0.05, 0.05),
                                   Vector2d(0.1, 0.1));
    trajectory_generator.addPathTo(point_3, Vector2d(0.1, 0.05),
                                   Vector2d(0.2, 0.2));
    trajectory_generator.addPathTo(point_1, Vector2d(0.05, 0.05),
                                   Vector2d(0.1, 0.1));

    auto reference = make_shared<Vector2d>();
    trajectory_generator.enableErrorTracking(reference, Vector2d(0.01, 0.01),
                                             true);

    trajectory_generator.computeTimings();

    Clock clock(SAMPLE_TIME);
    DataLogger logger("/tmp", clock.getTime(), true);

    logger.logExternalData("traj6d-vel", velocity_target->translation().data(),
                           3);
    logger.logExternalData("traj6d-pos", position_target->translation().data(),
                           3);
    logger.logExternalData("rob-pos", robot_position->translation().data(), 3);

    signal(SIGINT, sigint_handler);

    driver.enableSynchonous(true);
    driver.nextStep();
    driver.readTCPPose(position_target, ReferenceFrame::Base);

    trajectory_generator();

    bool end = false;
    while (not(_stop or end)) {
        if (driver.read()) {
            (*reference)[0] = robot_position->translation().x();
            (*reference)[1] = robot_position->translation().z();

            end = trajectory_generator();

            position_target->translation().x() =
                (*trajectory_generator.getPositionOutput())[0];
            position_target->translation().z() =
                (*trajectory_generator.getPositionOutput())[1];
            velocity_target->translation().x() =
                (*trajectory_generator.getVelocityOutput())[0];
            velocity_target->translation().z() =
                (*trajectory_generator.getVelocityOutput())[1];

            safety_controller();
            driver.send();

            clock();
            logger();
        }
        driver.nextStep();
    }

    cout << (end ? "End of the trajectory reached"
                 : "Trajectory generation interrupted")
         << endl;

    driver.enableSynchonous(false);
    driver.stop();

    return 0;
}
*/