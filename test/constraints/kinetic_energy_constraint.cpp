#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

TEST_CASE("Kinetic energy constraint") {

    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto mass = scalar::Mass{0.5};
    auto maximum_kinetic_energy = scalar::Energy{0.5};
    auto constant_vel = spatial::Velocity::Zero(robot.controlPointFrame());

    safety_controller.add<phri::KineticEnergyConstraint>(
        "kinetic energy constraint", mass, maximum_kinetic_energy);
    safety_controller.add<phri::VelocityProxy>("vel proxy", constant_vel);

    const auto& cp_velocity = robot.task().command().velocity();
    const auto& cp_total_velocity = robot.control().task().totalVelocity();

    auto current_kinetic_energy = [&]() -> scalar::Energy {
        return scalar::Energy::Kinetic(
            mass, scalar::Velocity{cp_velocity.linear().norm()});
    };

    // Step #1 : no velocity
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() == scalar::Energy{0.});

    // Step #2 : velocity 1 axis < max
    constant_vel.linear().x() = 0.1;
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() < maximum_kinetic_energy);

    // Step #3 : velocity 1 axis > max
    constant_vel.linear().x() = 2.;
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() == Approx(maximum_kinetic_energy));

    // Step #4 : velocity 3 axes < max
    constant_vel.linear().x() = 0.2;
    constant_vel.linear().y() = 0.1;
    constant_vel.linear().z() = 0.3;
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() < maximum_kinetic_energy);

    // Step #5 : velocity 3 axes > max
    constant_vel.linear().x() = 1.;
    constant_vel.linear().y() = 2.;
    constant_vel.linear().z() = 3.;
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() == Approx(maximum_kinetic_energy));

    // Step #6 : rotational velocity only
    constant_vel.setZero();
    constant_vel.angular().x() = 0.5;
    constant_vel.angular().y() = 0.4;
    constant_vel.angular().z() = 0.6;
    safety_controller.compute();

    REQUIRE(current_kinetic_energy() == scalar::Energy{0.});
}
