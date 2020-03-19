#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

TEST_CASE("Proxy generators") {
    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    robot.control().task().damping().diagonal().setConstant(10.);

    auto constant_vel = spatial::Velocity::Zero(robot.controlPointFrame());
    auto constant_force = spatial::Force::Zero(robot.controlPointFrame());

    const auto& cp_velocity = robot.task().command().velocity();

    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);
    safety_controller.add<phri::ForceProxy>("force proxy", constant_force);

    SECTION("no velocity, no force") {
        safety_controller.compute();

        REQUIRE(cp_velocity.isZero());
    }

    SECTION("velocity 1 axis, no force") {
        constant_vel.linear().x() = 0.5;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(0.5));
    }

    SECTION("velocity 2 axes, no force") {
        constant_vel.linear().x() = 1.;
        constant_vel.angular().x() = 1.;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(std::sqrt(2.)));
    }

    SECTION("no velocity, force 1 axis") {
        constant_vel.setZero();
        constant_force(2) = 20.;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(2.));
    }

    SECTION("no velocity, force 2 axes") {
        constant_force(2) = 10.;
        constant_force(5) = 10.;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(std::sqrt(2.)));
    }

    SECTION("velocity 3 axes, force 3 axes, separate axes") {
        constant_vel.setZero();
        constant_force.setZero();
        constant_vel.linear().x() = 1.;
        constant_vel.linear().z() = 1.;
        constant_vel.angular().y() = 1.;
        constant_force(1) = 10.;
        constant_force(3) = 10.;
        constant_force(5) = 10.;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(std::sqrt(6.)));
    }

    SECTION("velocity 3 axes, force 3 axes, mixed axes") {
        constant_vel.setZero();
        constant_force.setZero();
        constant_vel.linear().x() = 1.;
        constant_vel.angular().x() = 1.;
        constant_vel.angular().y() = 1.;
        constant_force(1) = 10.;
        constant_force(3) = 10.;
        constant_force(5) = 10.;

        safety_controller.compute();

        REQUIRE(cp_velocity.norm() == Approx(std::sqrt(8.)));
    }
}
