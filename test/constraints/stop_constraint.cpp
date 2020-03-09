#include <OpenPHRI/OpenPHRI.h>
#include <catch2/catch.hpp>
#include <pid/rpath.h>

#include "utils.h"

using namespace phri;
using namespace std;

TEST_CASE("Stop constraint") {

    auto robot = phri::Robot{"rob", // Robot's name
                             7};    // Robot's joint count

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    FrameAdapter::setTransform(FrameAdapter::world(),
                               AffineTransform::Identity(),
                               FrameAdapter::frame("end-effector"));

    robot.joints().state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());
    auto& ext_force = robot.task().state.wrench;
    auto activation_force_threshold = make_shared<double>(25.);
    auto deactivation_force_threshold = make_shared<double>(5.);

    auto& ext_torque = robot.joints().state.force;
    auto activation_torque_threshold = make_shared<VectorXd>(7);
    activation_torque_threshold->setConstant(5);
    auto deactivation_torque_threshold = make_shared<VectorXd>(7);
    deactivation_torque_threshold->setConstant(1);

    auto task_stop_constraint = make_shared<TaskEmergencyStopConstraint>(
        activation_force_threshold, deactivation_force_threshold);

    auto joint_stop_constraint = make_shared<JointEmergencyStopConstraint>(
        activation_torque_threshold, deactivation_torque_threshold);

    auto constant_vel =
        make_shared<spatial::Velocity>(FrameAdapter::frame("end-effector"));
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);
    auto constant_force_generator = make_shared<ExternalForce>();

    safety_controller.add("task stop constraint", task_stop_constraint);
    safety_controller.add("joint stop constraint", joint_stop_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);
    safety_controller.add("force proxy", constant_force_generator);

    const Vector6d& cp_velocity = robot.task().command.twist;
    const Vector6d& cp_total_velocity = robot.control().task.total_twist;

    // Step #1 : no velocity, no force
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #2 : velocity, no force
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : velocity, force < low
    ext_force.force().x() = 3;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #4 : velocity, low < force < max
    ext_force.force().x() = 15;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : velocity, force > max
    ext_force.force().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #6 : velocity, low < force < max
    ext_force.force().x() = 15;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #7 : velocity, force < low
    ext_force.force().x() = 4;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));

    // Step #8 : velocity, torque > high
    ext_torque(0) = 6;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #9 : velocity, torque and force > high
    ext_torque(0) = 6;
    ext_force.force().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #10 : velocity, torque < low and force > high
    ext_torque(0) = 0.5;
    ext_force.force().x() = 30;
    safety_controller.compute();

    REQUIRE(cp_velocity.isZero());

    // Step #11 : velocity, torque and force < low
    ext_torque(0) = 0.5;
    ext_force.force().x() = 2.;
    safety_controller.compute();

    REQUIRE(cp_velocity.isApprox(cp_total_velocity));
}
