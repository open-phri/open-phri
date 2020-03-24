#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

TEST_CASE("Joint acceleration constraint") {
    auto [robot, model, driver] = TestData{1.};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_acceleration = scalar::Acceleration{0.5};
    auto maximum_accelerations = vector::dyn::Acceleration{robot.jointCount()};
    maximum_accelerations.setConstant(maximum_acceleration);

    vector::dyn::Velocity dv_max =
        maximum_accelerations * scalar::Duration{robot.control().timeStep()};
    auto constant_vel = vector::dyn::Velocity(robot.jointCount());
    constant_vel.setZero();

    safety_controller.add<phri::JointAccelerationConstraint>(
        "acceleration constraint", maximum_accelerations);

    safety_controller.add<phri::JointVelocityProxy>("vel proxy", constant_vel);

    const auto& cp_velocity = robot.task().command().velocity();
    const auto& cp_total_velocity = robot.control().task().totalVelocity();

    auto estimated_acceleration = vector::dyn::Acceleration{robot.jointCount()};
    phri::Derivator<Eigen::VectorXd> derivator(
        robot.joints().command().velocity().value(),
        estimated_acceleration.value(), robot.control().timeStep());

    derivator.reset();

    // Step #1 : no acceleration
    safety_controller();
    derivator();

    REQUIRE(estimated_acceleration.isZero());

    // Step #2 : acceleration 1 axis < max
    constant_vel(0) += dv_max(0) * 0.5;
    safety_controller();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : acceleration 1 axis > max
    constant_vel(0) += dv_max(0) * 1.5;
    safety_controller();
    derivator();

    REQUIRE(estimated_acceleration(0) == Approx(maximum_accelerations(0)));

    // Step #4 : acceleration 3 axes < max
    constant_vel(0) += dv_max(0) * 0.1;
    constant_vel(1) += dv_max(1) * 0.2;
    constant_vel(2) += dv_max(2) * 0.3;
    safety_controller();
    derivator();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : acceleration 3 axes > max
    constant_vel = robot.joints().command().velocity();
    constant_vel(0) += dv_max(0) * 1.0;
    constant_vel(1) += dv_max(1) * 2.0;
    constant_vel(2) += dv_max(2) * 3.0;
    safety_controller();
    derivator();

    REQUIRE(estimated_acceleration(0) < maximum_accelerations(0));
    REQUIRE(estimated_acceleration(1) < maximum_accelerations(1));
    REQUIRE(estimated_acceleration(2) == Approx(maximum_accelerations(2)));
}
