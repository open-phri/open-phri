#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <pid/rpath.h>

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
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

    auto maximum_velocity = std::make_shared<double>(0.5);
    auto velocity_constraint =
        std::make_shared<phri::VelocityConstraint>(maximum_velocity);

    auto constant_vel = std::make_shared<phri::Twist>();
    auto constant_velocity_generator =
        std::make_shared<phri::VelocityProxy>(constant_vel);

    safety_controller.add("velocity constraint", velocity_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    auto cp_velocity =
        static_cast<const phri::Vector6d&>(robot.task.command.twist);
    auto cp_total_velocity =
        static_cast<const phri::Vector6d&>(robot.control.task.total_twist);

    // Step #1 : no velocity
    safety_controller.compute();

    assert_msg("Step #1", cp_velocity.isZero());

    // Step #2 : velocity 1 axis < max
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    assert_msg("Step #2", cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : velocity 1 axis > max
    constant_vel->translation().x() = 0.6;
    safety_controller.compute();

    assert_msg("Step #3", isClose(robot.task.command.twist.translation().norm(),
                                  *maximum_velocity));

    // Step #4 : velocity 3 axes < max
    constant_vel->translation().x() = 0.2;
    constant_vel->translation().y() = 0.1;
    constant_vel->translation().z() = 0.3;
    safety_controller.compute();

    assert_msg("Step #4", cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : velocity 3 axes > max
    constant_vel->translation().x() = 0.5;
    constant_vel->translation().y() = 0.4;
    constant_vel->translation().z() = 0.6;
    safety_controller.compute();

    assert_msg("Step #5", isClose(robot.task.command.twist.translation().norm(),
                                  *maximum_velocity));

    // Step #6 : rotational velocity only
    static_cast<phri::Vector6d&>(*constant_vel).setZero();
    constant_vel->rotation().x() = 0.5;
    constant_vel->rotation().y() = 0.4;
    constant_vel->rotation().z() = 0.6;
    safety_controller.compute();

    assert_msg("Step #6", cp_velocity.isApprox(cp_total_velocity));

    return 0;
}
