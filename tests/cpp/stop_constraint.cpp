#include <RSCL/RSCL.h>

using namespace RSCL;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
	return std::abs(v1-v2) < eps;
}

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());
	auto ext_force = make_shared<Vector6d>(Vector6d::Zero());
	auto activation_force_threshold = make_shared<double>(25.);
	auto deactivation_force_threshold = make_shared<double>(5.);

	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);
	auto tcp_velocity = safety_controller.getTCPVelocity();
	auto total_velocity = safety_controller.getTotalVelocity();

	auto stop_constraint = make_shared<StopConstraint>(ext_force, activation_force_threshold, deactivation_force_threshold);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);
	auto constant_force_generator = make_shared<ForceProxy>(ext_force);

	safety_controller.addConstraint("stop constraint", stop_constraint);
	safety_controller.addVelocityGenerator("vel proxy", constant_velocity_generator);
	safety_controller.addForceGenerator("force proxy", constant_force_generator);

	// Step #1 : no velocity, no force
	safety_controller.updateTCPVelocity();

	assert_msg("Step #1", tcp_velocity->isZero());

	// Step #2 : velocity, no force
	(*constant_vel)(0) = 0.2;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #2", tcp_velocity->isApprox(*total_velocity));

	// Step #3 : velocity, force < low
	(*ext_force)(0) = 3;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #3", tcp_velocity->isApprox(*total_velocity));

	// Step #4 : velocity, low < force < max
	(*ext_force)(0) = 15;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #4", tcp_velocity->isApprox(*total_velocity));

	// Step #5 : velocity, force > max
	(*ext_force)(0) = 30;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #5", tcp_velocity->isZero());

	// Step #6 : velocity, low < force < max
	(*ext_force)(0) = 15;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #6", tcp_velocity->isZero());

	// Step #7 : velocity, force < low
	(*ext_force)(0) = 4;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #7", tcp_velocity->isApprox(*total_velocity));

	return 0;
}
