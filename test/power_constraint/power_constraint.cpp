#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

#include <iostream>

using namespace phri;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

double power(TwistConstPtr velocity, Vector6dConstPtr force) {
    return force->dot(static_cast<Vector6d>(*velocity));
}

int main(int argc, char const* argv[]) {

    auto robot = make_shared<Robot>("rob", // Robot's name
                                    7);    // Robot's joint count

    *robot->controlPointDampingMatrix() *= 10.;

    auto safety_controller = SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_power = make_shared<double>(10);
    auto external_force = robot->controlPointExternalForce();
    auto power_constraint = make_shared<PowerConstraint>(maximum_power);

    auto constant_vel = make_shared<Twist>();
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

    safety_controller.add("power constraint", power_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    // Step #1 : no velocity
    safety_controller.compute();

    assert_msg(
        "Step #1",
        isClose(power(robot->controlPointVelocity(), external_force), 0.));

    // Step #2 : velocity 1 axis, no force
    constant_vel->translation().x() = 0.2;
    safety_controller.compute();

    assert_msg(
        "Step #2",
        isClose(power(robot->controlPointVelocity(), external_force), 0.));

    // Step #3 : velocity 1 axis, force same axis with opposite sign < max
    (*external_force)(0) = -10.;
    safety_controller.compute();

    assert_msg(
        "Step #3",
        isClose(power(robot->controlPointVelocity(), external_force), -2.));

    // Step #4 : velocity 1 axis, force same axis with same sign < max
    (*external_force)(0) = 10.;
    safety_controller.compute();

    assert_msg(
        "Step #4",
        isClose(power(robot->controlPointVelocity(), external_force), 2.));

    // Step #5 : velocity 1 axis, force same axis with opposite sign > max
    (*external_force)(0) = -100.;
    safety_controller.compute();

    assert_msg("Step #5",
               isClose(power(robot->controlPointVelocity(), external_force),
                       -*maximum_power));

    // Step #6 : velocity 1 axis, force same axis with same sign > max
    (*external_force)(0) = 100.;
    safety_controller.compute();

    assert_msg("Step #6", static_cast<Vector6d>(*robot->controlPointVelocity())
                              .isApprox(static_cast<Vector6d>(
                                  *robot->controlPointTotalVelocity())));

    return 0;
}
