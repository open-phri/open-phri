#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

TEST_CASE("Velocity constraint") {

    auto [robot, model, driver] = TestData{};

    // FrameAdapter::setTransform(FrameAdapter::world(),
    //                            AffineTransform::Identity(),
    //                            FrameAdapter::frame("end-effector"));

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_velocity = scalar::Velocity{0.5};
    auto constant_vel = spatial::Velocity::Zero(robot.controlPointFrame());

    safety_controller.add<phri::VelocityConstraint>("velocity constraint",
                                                    maximum_velocity);
    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);

    const auto& cp_velocity = robot.task().command().velocity();
    const auto& cp_total_velocity = robot.control().task().totalVelocity();

    // Step #1 : no velocity
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #2 : velocity 1 axis < max
    constant_vel.linear().x() = 0.2;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : velocity 1 axis > max
    constant_vel.linear().x() = 0.6;
    safety_controller.compute();

    REQUIRE(cp_velocity.linear().norm() == Approx(maximum_velocity.value()));

    // Step #4 : velocity 3 axes < max
    constant_vel.linear().x() = 0.2;
    constant_vel.linear().y() = 0.1;
    constant_vel.linear().z() = 0.3;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : velocity 3 axes > max
    constant_vel.linear().x() = 0.5;
    constant_vel.linear().y() = 0.4;
    constant_vel.linear().z() = 0.6;
    safety_controller.compute();

    REQUIRE(cp_velocity.linear().norm() == Approx(maximum_velocity.value()));

    // Step #6 : rotational velocity only
    constant_vel.setZero();
    constant_vel.angular().x() = 0.5;
    constant_vel.angular().y() = 0.4;
    constant_vel.angular().z() = 0.6;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));
}
