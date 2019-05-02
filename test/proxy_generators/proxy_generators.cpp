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

    *robot->controlPointDampingMatrix() *= 10.;

    auto safety_controller = SafetyController(robot);
    safety_controller.setVerbose(true);

    auto constant_vel = make_shared<Twist>();
    Vector6d& constant_vel_ref = *constant_vel;
    auto constant_force = make_shared<Vector6d>(Vector6d::Zero());

    auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);
    auto constant_force_generator = make_shared<ForceProxy>(constant_force);

    const Vector6d& cp_velocity = *robot->controlPointVelocity();

    safety_controller.add("vel proxy", constant_velocity_generator);
    safety_controller.add("force proxy", constant_force_generator);

    // Step #1 : no velocity, no force
    safety_controller.compute();

    assert_msg("Step #1", cp_velocity.isZero());

    // Step #2 : velocity 1 axis, no force
    constant_vel->translation().x() = 0.5;

    safety_controller.compute();

    assert_msg("Step #2", isClose(cp_velocity.norm(), 0.5));

    // Step #3 : velocity 2 axes, no force
    constant_vel->translation().x() = 1.;
    constant_vel->rotation().x() = 1.;

    safety_controller.compute();

    assert_msg("Step #3", isClose(cp_velocity.norm(), std::sqrt(2.)));

    // Step #4 : no velocity, force 1 axis
    constant_vel_ref.setZero();
    (*constant_force)(2) = 20.;

    safety_controller.compute();

    assert_msg("Step #4", isClose(cp_velocity.norm(), 2.));

    // Step #5 : no velocity, force 2 axes
    (*constant_force)(2) = 10.;
    (*constant_force)(5) = 10.;

    safety_controller.compute();

    assert_msg("Step #5", isClose(cp_velocity.norm(), std::sqrt(2.)));

    // Step #6 : velocity 3 axes, force 3 axes, separate axes
    constant_vel_ref.setZero();
    constant_force->setZero();
    constant_vel->translation().x() = 1.;
    constant_vel->translation().z() = 1.;
    constant_vel->rotation().y() = 1.;
    (*constant_force)(1) = 10.;
    (*constant_force)(3) = 10.;
    (*constant_force)(5) = 10.;

    safety_controller.compute();

    assert_msg("Step #6", isClose(cp_velocity.norm(), std::sqrt(6.)));

    // Step #7 : velocity 3 axes, force 3 axes, mixed axes
    constant_vel_ref.setZero();
    constant_force->setZero();
    constant_vel->translation().x() = 1.;
    constant_vel->rotation().x() = 1.;
    constant_vel->rotation().y() = 1.;
    (*constant_force)(1) = 10.;
    (*constant_force)(3) = 10.;
    (*constant_force)(5) = 10.;

    safety_controller.compute();

    assert_msg("Step #7", isClose(cp_velocity.norm(), std::sqrt(8.)));

    return 0;
}
