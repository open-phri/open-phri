#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

TEST_CASE("Power constraint") {
    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    // FrameAdapter::setTransform(FrameAdapter::world(),
    //                            AffineTransform::Identity(),
    //                            FrameAdapter::frame("end-effector"));

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    robot.control().task().damping().diagonal().setConstant(10);

    auto maximum_power = scalar::Power{10};
    auto& external_wrench = driver.taskState().force();
    auto& command_twist = robot.task().command().velocity();

    auto constant_vel = spatial::Velocity{robot.controlPointFrame()};

    safety_controller.add<phri::PowerConstraint>("power constraint",
                                                 maximum_power);
    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);

    // Step #1 : no velocity
    safety_controller.compute();

    REQUIRE(external_wrench.dot(command_twist).value() ==
            Approx(0.).margin(1e-9));

    // Step #2 : velocity 1 axis, no force
    constant_vel.linear().x() = 0.2;
    safety_controller.compute();

    REQUIRE(external_wrench.dot(command_twist).value() ==
            Approx(0.).margin(1e-9));

    // Step #3 : velocity 1 axis, force same axis with opposite sign < max
    external_wrench.linear().x() = -10.;
    safety_controller.compute();

    REQUIRE(external_wrench.dot(command_twist).value() == Approx(-2.));

    // Step #4 : velocity 1 axis, force same axis with same sign < max
    external_wrench.linear().x() = 10.;
    safety_controller.compute();

    REQUIRE(external_wrench.dot(command_twist).value() == Approx(2.));

    // Step #5 : velocity 1 axis, force same axis with opposite sign > max
    external_wrench.linear().x() = -100.;
    safety_controller.compute();

    REQUIRE(external_wrench.dot(command_twist).value() ==
            Approx(-maximum_power.value()));

    // Step #6 : velocity 1 axis, force same axis with same sign > max
    external_wrench.linear().x() = 100.;
    safety_controller.compute();

    REQUIRE(command_twist.isApprox(robot.control().task().totalVelocity()));
}
