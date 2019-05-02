#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <pid/rpath.h>
#include <iostream>

using namespace phri;
using namespace std;

int main(int argc, char const* argv[]) {

    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    robot.joints.state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto potential_field_generator = make_shared<PotentialFieldGenerator>();
    potential_field_generator->setVerbose(true);

    safety_controller.add("potential field", potential_field_generator);

    auto obs_pos = make_shared<Pose>();
    auto obstacle = make_shared<PotentialFieldObject>(
        PotentialFieldType::Repulsive,
        make_shared<double>(10.), // gain
        make_shared<double>(0.2), // threshold distance
        obs_pos);

    auto tgt_pos = make_shared<Pose>();
    auto target = make_shared<PotentialFieldObject>(
        PotentialFieldType::Attractive,
        make_shared<double>(10.), // gain
        make_shared<double>(
            std::numeric_limits<double>::infinity()), // threshold distance
        tgt_pos);

    // Step #1 : add
    bool ok;
    ok = potential_field_generator->add("obstacle", obstacle);
    assert_msg("Step #1", ok == true);

    // Step #2 : re-add, force=false
    ok = potential_field_generator->add("obstacle", obstacle);
    assert_msg("Step #2", ok == false);

    // Step #3 : re-add, force=true
    ok = potential_field_generator->add("obstacle", obstacle, true);
    assert_msg("Step #3", ok == true);

    // Step #4 : remove
    ok = potential_field_generator->remove("obstacle");
    assert_msg("Step #4", ok == true);

    // Step #5 : re-remove
    try {
        potential_field_generator->remove("obstacle");
        assert_msg("Step #5", false);
    } catch (std::domain_error& err) {
        std::cerr << "Expected exception: " << err.what() << std::endl;
    }

    // Step #6 : get
    potential_field_generator->add("obstacle", obstacle);
    auto obj = potential_field_generator->get("obstacle");
    assert_msg("Step #6", obj == obstacle);

    // Step #7 : 1 obstacle > threshold distance
    obs_pos->translation().x() = 0.3;
    safety_controller.compute();
    assert_msg("Step #7", robot.task.command.twist.vector().isZero());

    // Step #8 : 1 obstacle < threshold distance
    obs_pos->translation().x() = 0.1;
    safety_controller.compute();
    assert_msg("Step #8", robot.task.command.twist.translation().dot(
                              obs_pos->translation()) < 0.);

    // Step #9 : 1 target
    ok = potential_field_generator->remove("obstacle");
    potential_field_generator->add("target", target);
    tgt_pos->translation().x() = 0.1;
    tgt_pos->translation().y() = 0.2;
    safety_controller.compute();
    assert_msg("Step #9", robot.task.command.twist.translation().dot(
                              tgt_pos->translation()) > 0.);

    return 0;
}
