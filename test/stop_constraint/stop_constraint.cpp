#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

using namespace phri;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

int main(int argc, char const* argv[]) {

    auto robot = make_shared<Robot>("rob", // Robot's name
                                    7);    // Robot's joint count

    auto safety_controller = SafetyController(robot);
    safety_controller.setVerbose(true);

    auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());
    auto ext_force = robot->controlPointExternalForce();
    auto activation_force_threshold = make_shared<double>(25.);
    auto deactivation_force_threshold = make_shared<double>(5.);

    auto ext_torque = robot->jointExternalTorque();
    auto activation_torque_threshold = make_shared<double>(5.);
    auto deactivation_torque_threshold = make_shared<double>(1.);

    auto stop_constraint = make_shared<EmergencyStopConstraint>(
        EmergencyStopConstraint::CheckBoth, // Check both external forces and
                                            // external joint torques
        activation_force_threshold, deactivation_force_threshold,
        activation_torque_threshold, deactivation_torque_threshold);

    auto constant_vel = make_shared<Twist>();
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);
    auto constant_force_generator = make_shared<ForceProxy>(ext_force);

    safety_controller.add("stop constraint", stop_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);
    safety_controller.add("force proxy", constant_force_generator);

    const Vector6d& cp_velocity = *robot->controlPointVelocity();
    const Vector6d& cp_total_velocity = *robot->controlPointTotalVelocity();

    // Step #1 : no velocity, no force
    safety_controller.compute();

    assert_msg("Step #1", cp_velocity.isZero());

    // Step #2 : velocity, no force
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    assert_msg("Step #2", cp_velocity.isApprox(cp_total_velocity));

    // Step #3 : velocity, force < low
    (*ext_force)(0) = 3;
    safety_controller.compute();

    assert_msg("Step #3", cp_velocity.isApprox(cp_total_velocity));

    // Step #4 : velocity, low < force < max
    (*ext_force)(0) = 15;
    safety_controller.compute();

    assert_msg("Step #4", cp_velocity.isApprox(cp_total_velocity));

    // Step #5 : velocity, force > max
    (*ext_force)(0) = 30;
    safety_controller.compute();

    assert_msg("Step #5", cp_velocity.isZero());

    // Step #6 : velocity, low < force < max
    (*ext_force)(0) = 15;
    safety_controller.compute();

    assert_msg("Step #6", cp_velocity.isZero());

    // Step #7 : velocity, force < low
    (*ext_force)(0) = 4;
    safety_controller.compute();

    assert_msg("Step #7", cp_velocity.isApprox(cp_total_velocity));

    // Step #8 : velocity, torque > high
    (*ext_torque)(0) = 6;
    safety_controller.compute();

    assert_msg("Step #8", cp_velocity.isZero());

    // Step #9 : velocity, torque and force > high
    (*ext_torque)(0) = 6;
    (*ext_force)(0) = 30;
    safety_controller.compute();

    assert_msg("Step #9", cp_velocity.isZero());

    // Step #10 : velocity, torque < low and force > high
    (*ext_torque)(0) = 0.5;
    (*ext_force)(0) = 30;
    safety_controller.compute();

    assert_msg("Step #10", cp_velocity.isZero());

    // Step #11 : velocity, torque and force < low
    (*ext_torque)(0) = 0.5;
    (*ext_force)(0) = 2.;
    safety_controller.compute();

    assert_msg("Step #11", cp_velocity.isApprox(cp_total_velocity));

    return 0;
}
