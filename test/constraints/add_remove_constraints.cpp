#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

using namespace phri;
using namespace std;

TEST_CASE("add/remove constraints") {
    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    robot.joints.state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_velocity = make_shared<double>(0.1);
    auto velocity_constraint =
        make_shared<VelocityConstraint>(maximum_velocity);

    REQUIRE(safety_controller.add("velocity limit", velocity_constraint));

    REQUIRE(safety_controller.getConstraint("velocity limit") ==
            velocity_constraint);

    REQUIRE_FALSE(safety_controller.add("velocity limit",
                                        VelocityConstraint(maximum_velocity)));

    REQUIRE(safety_controller.add("velocity limit",
                                  std::move(*velocity_constraint), true));

    REQUIRE(safety_controller.removeConstraint("velocity limit"));

    REQUIRE_THROWS(safety_controller.removeConstraint("velocity limit"));

    REQUIRE_THROWS(safety_controller.getConstraint("velocity limit"));
}
