#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

using namespace phri;

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

int main(int argc, char const* argv[]) {

    auto robot = make_shared<Robot>("rob", // Robot's name
                                    7);    // Robot's joint count

    auto safety_controller = SafetyController(robot);
    safety_controller.setVerbose(true);

    auto maximum_velocity = make_shared<double>(0.5);
    auto velocity_constraint =
        make_shared<VelocityConstraint>(maximum_velocity);

    auto constant_vel = make_shared<Twist>();
    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

    safety_controller.add("velocity constraint", velocity_constraint);
    safety_controller.add("vel proxy", constant_velocity_generator);

    const Vector6d& cp_velocity = *robot->controlPointVelocity();
    const Vector6d& cp_total_velocity = *robot->controlPointTotalVelocity();

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

    assert_msg("Step #3",
               isClose(robot->controlPointVelocity()->translation().norm(),
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

    assert_msg("Step #5",
               isClose(robot->controlPointVelocity()->translation().norm(),
                       *maximum_velocity));

    // Step #6 : rotational velocity only
    static_cast<Vector6d&>(*constant_vel).setZero();
    constant_vel->rotation().x() = 0.5;
    constant_vel->rotation().y() = 0.4;
    constant_vel->rotation().z() = 0.6;
    safety_controller.compute();

    assert_msg("Step #6", cp_velocity.isApprox(cp_total_velocity));

    return 0;
}
