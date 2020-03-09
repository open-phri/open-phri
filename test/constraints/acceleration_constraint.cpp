#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

using namespace phri;
using namespace std;

TEST_CASE("Acceleration constraint") {
    using namespace spatial::literals;

    auto robot = phri::Robot{"tcp"_frame, "base"_frame, "rob", 7};

    robot.control().time_step = 1.;

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    FrameAdapter::setTransform(FrameAdapter::world(),
                               AffineTransform::Identity(),
                               FrameAdapter::frame("end-effector"));

    robot.joints().state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_acceleration = 0.5;
    auto dv_max = maximum_acceleration * robot.control().time_step;
    auto constant_vel = std::make_shared<spatial::Velocity>("tcp"_frame);

    safety_controller.add("acceleration constraint",
                          phri::AccelerationConstraint(maximum_acceleration));

    safety_controller.add("vel proxy", phri::VelocityProxy(constant_vel));

    const Eigen::Vector6d& cp_velocity = robot.task().command.twist;
    const Eigen::Vector6d& cp_total_velocity = robot.control().task.total_twist;

    auto vnorm = std::make_shared<double>(0.);
    auto anorm = std::make_shared<double>(0.);
    phri::Derivator<double> derivator(vnorm, anorm, robot.control().time_step);

    derivator.reset();

    // Step #1 : no acceleration
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(isClose(std::abs(*anorm), 0.));

    // Step #2 : acceleration 1 axis < max
    constant_vel->translation().x() += 0.5 * dv_max;
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : acceleration 1 axis > max
    constant_vel->translation().x() += 1.5 * dv_max;
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(isClose(std::abs(*anorm), maximum_acceleration));

    // Step #4 : acceleration 3 axes < max
    constant_vel->translation().x() += 0.1 * dv_max;
    constant_vel->translation().y() += 0.2 * dv_max;
    constant_vel->translation().z() += 0.3 * dv_max;
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : acceleration 3 axes > max
    *constant_vel = cp_velocity;
    constant_vel->translation().x() += 1 * dv_max;
    constant_vel->translation().y() += 2 * dv_max;
    constant_vel->translation().z() += 3 * dv_max;
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(isClose(std::abs(*anorm), maximum_acceleration));

    // Step #6 : rotational acceleration only
    constant_vel->vector().setZero();
    constant_vel->rotation().x() = 0.5;
    constant_vel->rotation().y() = 0.4;
    constant_vel->rotation().z() = 0.6;
    safety_controller();
    *vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));
}
