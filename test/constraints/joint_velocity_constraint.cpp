#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

TEST_CASE("Joint velocity constraint") {
    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_velocities = vector::dyn::Velocity{robot.jointCount()};
    maximum_velocities << 1., 2., 3., 4., 5., 6., 7.;

    auto constant_vel = vector::dyn::Velocity{robot.jointCount()};
    constant_vel.setZero();

    safety_controller.add<phri::JointVelocityConstraint>("velocity constraint",
                                                         maximum_velocities);
    safety_controller.add<phri::JointVelocityProxy>("vel proxy", constant_vel);

    // Step #1 : no velocity
    safety_controller.compute();

    REQUIRE(robot.joints().command().velocity().isZero());

    // Step #2 : velocity 1 axis < max
    constant_vel(0) = 0.5;
    safety_controller.compute();

    REQUIRE(robot.joints().command().velocity().isApprox(
        robot.control().joints().totalVelocity()));

    // Step #3 : velocity 1 axis > max
    constant_vel(0) = 1.5;
    safety_controller.compute();

    REQUIRE(
        isLessOrEqual(robot.joints().command().velocity(), maximum_velocities));

    // Step #4 : velocity 3 axes < max
    constant_vel(0) = 0.5;
    constant_vel(1) = 1.;
    constant_vel(2) = 1.5;
    safety_controller.compute();

    REQUIRE(robot.task().command().velocity().isApprox(
        robot.control().task().totalVelocity()));

    // Step #5 : velocity 3 axes > max
    constant_vel(0) = 1.5;
    constant_vel(1) = 2.5;
    constant_vel(2) = 3.5;
    safety_controller.compute();

    REQUIRE(
        isLessOrEqual(robot.joints().command().velocity(), maximum_velocities));
}
