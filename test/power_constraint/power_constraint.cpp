#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <pid/rpath.h>
#include <iostream>

using namespace phri;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

double power(const Twist& velocity, const Wrench& force) {
    return force.vector().dot(velocity.vector());
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

    robot.control.task.damping.setConstant(10);

    auto maximum_power = make_shared<double>(10);
    auto& external_wrench = robot.task.state.wrench;
    auto& command_twist = robot.task.command.twist;
    auto power_constraint = make_shared<PowerConstraint>(maximum_power);

    auto constant_vel = make_shared<Twist>();
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

    safety_controller.add("power constraint", power_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    // Step #1 : no velocity
    safety_controller.compute();

    assert_msg("Step #1", isClose(power(command_twist, external_wrench), 0.));

    // Step #2 : velocity 1 axis, no force
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    assert_msg("Step #2", isClose(power(command_twist, external_wrench), 0.));

    // Step #3 : velocity 1 axis, force same axis with opposite sign < max
    external_wrench.force().x() = -10.;
    safety_controller.compute();

    assert_msg("Step #3", isClose(power(command_twist, external_wrench), -2.));

    // Step #4 : velocity 1 axis, force same axis with same sign < max
    external_wrench.force().x() = 10.;
    safety_controller.compute();

    assert_msg("Step #4", isClose(power(command_twist, external_wrench), 2.));

    // Step #5 : velocity 1 axis, force same axis with opposite sign > max
    external_wrench.force().x() = -100.;
    safety_controller.compute();

    assert_msg("Step #5",
               isClose(power(command_twist, external_wrench), -*maximum_power));

    // Step #6 : velocity 1 axis, force same axis with same sign > max
    external_wrench.force().x() = 100.;
    safety_controller.compute();

    assert_msg("Step #6", command_twist.vector().isApprox(
                              robot.control.task.total_twist.vector()));

    return 0;
}
