#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <pid/rpath.h>

using namespace phri;
using namespace std;

bool isLessOrEqual(VectorXd v1, VectorXd v2) {
    bool ok = true;
    for (size_t i = 0; i < v1.size(); ++i) {
        ok &= std::abs(v1(i)) <= std::abs(v2(i));
    }
    return ok;
}

int main(int argc, char const* argv[]) {

    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    robot.joints.state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_velocities = make_shared<VectorXd>(7);
    *maximum_velocities << 1., 2., 3., 4., 5., 6., 7.;
    auto velocity_constraint =
        make_shared<JointVelocityConstraint>(maximum_velocities);

    auto constant_vel = make_shared<VectorXd>(7);
    constant_vel->setZero();
    auto constant_velocity_generator =
        make_shared<JointVelocityProxy>(constant_vel);

    safety_controller.add("velocity constraint", velocity_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    // Step #1 : no velocity
    safety_controller.compute();

    assert_msg("Step #1", robot.joints.command.velocity.isZero());

    // Step #2 : velocity 1 axis < max
    (*constant_vel)(0) = 0.5;
    safety_controller.compute();

    assert_msg("Step #2", robot.joints.command.velocity.isApprox(
                              robot.control.joints.total_velocity));

    // Step #3 : velocity 1 axis > max
    (*constant_vel)(0) = 1.5;
    safety_controller.compute();

    assert_msg("Step #3", isLessOrEqual(robot.joints.command.velocity,
                                        *maximum_velocities));

    // Step #4 : velocity 3 axes < max
    (*constant_vel)(0) = 0.5;
    (*constant_vel)(1) = 1.;
    (*constant_vel)(2) = 1.5;
    safety_controller.compute();

    assert_msg("Step #4", robot.task.command.twist.vector().isApprox(
                              robot.control.task.total_twist.vector()));

    // Step #5 : velocity 3 axes > max
    (*constant_vel)(0) = 1.5;
    (*constant_vel)(1) = 2.5;
    (*constant_vel)(2) = 3.5;
    safety_controller.compute();

    assert_msg("Step #5", isLessOrEqual(robot.joints.command.velocity,
                                        *maximum_velocities));

    return 0;
}
