#undef NDEBUG

#include <RSCL/RSCL.h>

#include <iostream>

using namespace RSCL;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
	return std::abs(v1-v2) < eps;
}

double power(Vector6dConstPtr velocity, Vector6dConstPtr force) {
	return force->dot(*velocity);
}

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 10.);
	auto maximum_power = make_shared<double>(10);
	auto external_force = make_shared<Vector6d>(Vector6d::Zero());

	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);
	auto tcp_velocity = safety_controller.getTCPVelocity();
	auto total_velocity = safety_controller.getTotalVelocity();

	auto power_constraint = make_shared<PowerConstraint>(external_force, maximum_power);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

	safety_controller.add("power constraint", power_constraint);
	safety_controller.add("vel proxy", constant_velocity_generator);

	// Step #1 : no velocity
	safety_controller.updateTCPVelocity();

	assert_msg("Step #1", isClose(power(tcp_velocity, external_force), 0.));

	// Step #2 : velocity 1 axis, no force
	(*constant_vel)(0) = 0.2;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #2", isClose(power(tcp_velocity, external_force), 0.));

	// Step #3 : velocity 1 axis, force same axis with opposite sign < max
	(*external_force)(0) = -10.;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #3", isClose(power(tcp_velocity, external_force), -2.));

	// Step #4 : velocity 1 axis, force same axis with same sign < max
	(*external_force)(0) = 10.;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #4", isClose(power(tcp_velocity, external_force), 2.));

	// Step #5 : velocity 1 axis, force same axis with opposite sign > max
	(*external_force)(0) = -100.;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #5", isClose(power(tcp_velocity, external_force), -*maximum_power));

	// Step #6 : velocity 1 axis, force same axis with same sign > max
	(*external_force)(0) = 100.;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #6", tcp_velocity->isApprox(*total_velocity));

	return 0;
}
