#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>

#include "utils.h"

#include <iostream>

namespace phri {
bool operator==(const PotentialFieldObject& obj1,
                const PotentialFieldObject& obj2) {
    return obj1.type() == obj2.type() and obj1.getGain() == obj2.getGain() and
           obj1.getObjectPosition() == obj2.getObjectPosition() and
           obj1.getThresholdDistance() == obj2.getThresholdDistance();
}
} // namespace phri

TEST_CASE("Potential field generator") {
    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    safety_controller.add<phri::PotentialFieldGenerator>("potential field");
    auto& potential_field_generator =
        *safety_controller.get<phri::PotentialFieldGenerator>(
            "potential field");
    potential_field_generator.setVerbose(true);

    auto obs_pos = spatial::Position::Zero(robot.controlPointFrame());
    auto obstacle =
        phri::PotentialFieldObject{phri::PotentialFieldType::Repulsive,
                                   10., // gain
                                   0.2, // threshold distance
                                   obs_pos};

    auto tgt_pos = spatial::Position::Zero(robot.controlPointFrame());
    auto target = phri::PotentialFieldObject{
        phri::PotentialFieldType::Attractive,
        10.,                                     // gain
        std::numeric_limits<double>::infinity(), // threshold distance
        tgt_pos};

    // Step #1 : add
    bool ok;
    ok = potential_field_generator.add("obstacle", obstacle);
    REQUIRE(ok == true);

    // Step #2 : re-add, force=false
    ok = potential_field_generator.add("obstacle", obstacle);
    REQUIRE(ok == false);

    // Step #3 : re-add, force=true
    ok = potential_field_generator.add("obstacle", obstacle, true);
    REQUIRE(ok == true);

    // Step #4 : remove
    ok = potential_field_generator.remove("obstacle");
    REQUIRE(ok == true);

    // Step #5 : re-remove
    REQUIRE_THROWS(potential_field_generator.remove("obstacle"));

    // Step #6 : get
    potential_field_generator.add("obstacle", obstacle);
    auto obj = potential_field_generator.get("obstacle");
    REQUIRE(obj == obstacle);

    // Step #7 : 1 obstacle > threshold distance
    obs_pos.linear().x() = 0.3;
    safety_controller.compute();
    REQUIRE(robot.task().command().velocity().isZero());

    // Step #8 : 1 obstacle < threshold distance
    obs_pos.linear().x() = 0.1;
    safety_controller.compute();
    REQUIRE(robot.task().command().velocity().linear().dot(obs_pos.linear()) <
            0.);

    // Step #9 : 1 target
    ok = potential_field_generator.remove("obstacle");
    potential_field_generator.add("target", target);
    tgt_pos.linear().x() = 0.1;
    tgt_pos.linear().y() = 0.2;
    safety_controller.compute();
    REQUIRE(robot.task().command().velocity().linear().dot(tgt_pos.linear()) >
            0.);
}
