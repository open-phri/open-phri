#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

using namespace phri;
using namespace std;

TEST_CASE("Power constraint") {

    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    FrameAdapter::setTransform(FrameAdapter::world(),
                               AffineTransform::Identity(),
                               FrameAdapter::frame("end-effector"));

    robot.joints.state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    robot.control.task.damping.setConstant(10);

    auto maximum_power = make_shared<double>(10);
    auto& external_wrench = robot.task.state.wrench;
    auto& command_twist = robot.task.command.twist;
    auto power_constraint = make_shared<PowerConstraint>(maximum_power);

    auto constant_vel = make_shared<Twist>(FrameAdapter::frame("end-effector"));
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

    safety_controller.add("power constraint", power_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    // Step #1 : no velocity
    safety_controller.compute();

    REQUIRE(isClose(power(command_twist, external_wrench), 0.));

    // Step #2 : velocity 1 axis, no force
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    REQUIRE(isClose(power(command_twist, external_wrench), 0.));

    // Step #3 : velocity 1 axis, force same axis with opposite sign < max
    external_wrench.force().x() = -10.;
    safety_controller.compute();

    REQUIRE(isClose(power(command_twist, external_wrench), -2.));

    // Step #4 : velocity 1 axis, force same axis with same sign < max
    external_wrench.force().x() = 10.;
    safety_controller.compute();

    REQUIRE(isClose(power(command_twist, external_wrench), 2.));

    // Step #5 : velocity 1 axis, force same axis with opposite sign > max
    external_wrench.force().x() = -100.;
    safety_controller.compute();

    REQUIRE(isClose(power(command_twist, external_wrench), -*maximum_power));

    // Step #6 : velocity 1 axis, force same axis with same sign > max
    external_wrench.force().x() = 100.;
    safety_controller.compute();

    REQUIRE(command_twist.vector().isApprox(
        robot.control.task.total_twist.vector()));
}
