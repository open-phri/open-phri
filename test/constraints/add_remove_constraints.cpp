#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

TEST_CASE("add/remove constraints") {
    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    robot.joints.state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    SECTION("default construction") {
        REQUIRE_NOTHROW([&] {
            safety_controller.add("cstr", phri::VelocityConstraint());
            auto cstr = safety_controller.get<phri::VelocityConstraint>("cstr");
            cstr->maximumVelocity() = 1.;
        }());
    }

    SECTION("construction with shared_ptr") {
        REQUIRE_NOTHROW([&] {
            auto lim = std::make_shared<double>(1.);
            safety_controller.add("cstr", phri::VelocityConstraint(lim));
            auto cstr = safety_controller.get<phri::VelocityConstraint>("cstr");
            REQUIRE(cstr->maximumVelocity() == 1.);
        }());

        REQUIRE_THROWS(phri::VelocityConstraint(std::shared_ptr<double>()));
    }

    SECTION("construction with non-const lvalue reference") {
        REQUIRE_NOTHROW([&] {
            auto lim{1.};
            safety_controller.add("cstr", phri::VelocityConstraint(lim));
            auto cstr = safety_controller.get<phri::VelocityConstraint>("cstr");
            REQUIRE(cstr->maximumVelocity() == 1.);
            lim = 2.;
            REQUIRE(cstr->maximumVelocity() == 2.);
        }());
    }

    SECTION("construction with const lvalue reference") {
        REQUIRE_NOTHROW([&] {
            auto lim{1.};
            safety_controller.add(
                "cstr", phri::VelocityConstraint(phri::as_const(lim)));
            auto cstr = safety_controller.get<phri::VelocityConstraint>("cstr");
            REQUIRE(cstr->maximumVelocity() == 1.);
            lim = 2.;
            REQUIRE(cstr->maximumVelocity() == 1.);
        }());
    }

    SECTION("construction with rvalue reference") {
        REQUIRE_NOTHROW([&] {
            safety_controller.add("cstr", phri::VelocityConstraint(1.));
            auto cstr = safety_controller.get<phri::VelocityConstraint>("cstr");
            REQUIRE(cstr->maximumVelocity() == 1.);
        }());
    }

    SECTION("add/get/remove same constraint") {
        auto maximum_velocity = std::make_shared<double>(0.1);
        auto velocity_constraint =
            std::make_shared<phri::VelocityConstraint>(maximum_velocity);

        REQUIRE(safety_controller.add("velocity limit", velocity_constraint));

        REQUIRE(safety_controller.getConstraint("velocity limit") ==
                velocity_constraint);

        REQUIRE_FALSE(safety_controller.add(
            "velocity limit", phri::VelocityConstraint(maximum_velocity)));

        REQUIRE(safety_controller.add("velocity limit",
                                      std::move(*velocity_constraint), true));

        REQUIRE(safety_controller.removeConstraint("velocity limit"));

        REQUIRE_THROWS(safety_controller.removeConstraint("velocity limit"));

        REQUIRE_THROWS(safety_controller.getConstraint("velocity limit"));
    }
}
