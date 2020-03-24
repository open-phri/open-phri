#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

TEST_CASE("Acceleration constraint") {
    auto [robot, model, driver] = TestData{1.};

    // FrameAdapter::setTransform(FrameAdapter::world(),
    //                            AffineTransform::Identity(),
    //                            FrameAdapter::frame("end-effector"));

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    scalar::Acceleration maximum_acceleration{0.5};
    scalar::Velocity dv_max =
        maximum_acceleration * scalar::Duration{robot.control().timeStep()};
    auto constant_vel = spatial::Velocity::Zero(robot.controlPointFrame());

    safety_controller.add<phri::AccelerationConstraint>(
        "acceleration constraint", maximum_acceleration);

    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);

    const auto& cp_velocity = robot.task().command().velocity();
    const auto& cp_total_velocity = robot.control().task().totalVelocity();

    auto vnorm = double{0.};
    auto anorm = double{0.};
    phri::Derivator<double> derivator(vnorm, anorm, robot.control().timeStep());

    derivator.reset();

    std::cout << derivator.input() << " - " << derivator.output() << std::endl;

    // Step #1 : no acceleration
    safety_controller();
    vnorm = cp_velocity.norm();
    std::cout << derivator.input() << " - " << derivator.output() << std::endl;
    derivator();
    std::cout << derivator.input() << " - " << derivator.output() << std::endl;

    REQUIRE(std::abs(anorm) == Approx(0.));

    // Step #2 : acceleration 1 axis < max
    constant_vel.linear().x() += dv_max.value() * 0.5;
    safety_controller();
    vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : acceleration 1 axis > max
    constant_vel.linear().x() += dv_max.value() * 1.5;
    safety_controller();
    vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(std::abs(anorm) == Approx(maximum_acceleration));

    // Step #4 : acceleration 3 axes < max
    constant_vel.linear().x() += dv_max.value() * 0.1;
    constant_vel.linear().y() += dv_max.value() * 0.2;
    constant_vel.linear().z() += dv_max.value() * 0.3;
    safety_controller();
    vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : acceleration 3 axes > max
    constant_vel = cp_velocity;
    constant_vel.linear().x() += dv_max.value() * 1.0;
    constant_vel.linear().y() += dv_max.value() * 2.0;
    constant_vel.linear().z() += dv_max.value() * 3.0;
    safety_controller();
    vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(std::abs(anorm) == Approx(maximum_acceleration));

    // Step #6 : rotational acceleration only
    constant_vel.setZero();
    constant_vel.angular().x() = 0.5;
    constant_vel.angular().y() = 0.4;
    constant_vel.angular().z() = 0.6;
    safety_controller();
    vnorm = cp_velocity.norm();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));
}
