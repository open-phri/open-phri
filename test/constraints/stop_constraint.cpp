#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

TEST_CASE("Stop constraint") {

    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    // FrameAdapter::setTransform(FrameAdapter::world(),
    //                            AffineTransform::Identity(),
    //                            FrameAdapter::frame("end-effector"));

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto& ext_force = driver.taskState().force();
    auto activation_force_threshold = scalar::Force{25.};
    auto deactivation_force_threshold = scalar::Force{5.};

    auto& ext_torque = driver.jointState().force();
    auto activation_torque_threshold = vector::dyn::Force{7};
    activation_torque_threshold.setConstant(5);
    auto deactivation_torque_threshold = vector::dyn::Force{7};
    deactivation_torque_threshold.setConstant(1);

    auto constant_vel = spatial::Velocity::Zero(robot.controlPointFrame());

    safety_controller.add<phri::TaskEmergencyStopConstraint>(
        "task stop constraint", activation_force_threshold,
        deactivation_force_threshold);
    safety_controller.add<phri::JointEmergencyStopConstraint>(
        "joint stop constraint", activation_torque_threshold,
        deactivation_torque_threshold);
    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);
    safety_controller.add<phri::ExternalForce>("force proxy");

    const auto& cp_velocity = robot.task().command().velocity();
    const auto& cp_total_velocity = robot.control().task().totalVelocity();

    // Step #1 : no velocity, no force
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #2 : velocity, no force
    constant_vel.linear().x() = 0.2;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : velocity, force < low
    ext_force.linear().x() = 3;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #4 : velocity, low < force < max
    ext_force.linear().x() = 15;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : velocity, force > max
    ext_force.linear().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #6 : velocity, low < force < max
    ext_force.linear().x() = 15;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #7 : velocity, force < low
    ext_force.linear().x() = 4;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #8 : velocity, torque > high
    ext_torque(0) = 6;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #9 : velocity, torque and force > high
    ext_torque(0) = 6;
    ext_force.linear().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #10 : velocity, torque < low and force > high
    ext_torque(0) = 0.5;
    ext_force.linear().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #11 : velocity, torque and force < low
    ext_torque(0) = 0.5;
    ext_force.linear().x() = 2.;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));
}
