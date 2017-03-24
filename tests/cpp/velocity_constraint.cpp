#include <safety_controller.h>
#include <velocity_generators.h>
#include <force_generators.h>
#include <constraints.h>

using namespace RSCL;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
	return std::abs(v1-v2) < eps;
}

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 10.);
	auto maximum_velocity = make_shared<double>(0.5);

	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);
	auto tcp_velocity = safety_controller.getTCPVelocity();
	auto total_velocity = safety_controller.getTotalVelocity();

	auto velocity_constraint = make_shared<VelocityConstraint>(total_velocity, maximum_velocity);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

	safety_controller.addConstraint("velocity constraint", velocity_constraint);
	safety_controller.addVelocityGenerator("const vel", constant_velocity_generator);

	auto& translation_velocity = tcp_velocity->block<3,1>(0,0);

	// Step #1 : no velocity
	safety_controller.updateTCPVelocity();

	assert_msg("Step #1", tcp_velocity->isZero());

	// Step #2 : velocity 1 axis < max
	(*constant_vel)(0) = 0.2;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #2", tcp_velocity->isApprox(*total_velocity));

	// Step #3 : velocity 1 axis > max
	(*constant_vel)(0) = 0.6;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #3", isClose(translation_velocity.norm(), *maximum_velocity));

	// Step #4 : velocity 3 axes < max
	(*constant_vel)(0) = 0.2;
	(*constant_vel)(1) = 0.1;
	(*constant_vel)(2) = 0.3;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #4", tcp_velocity->isApprox(*total_velocity));

	// Step #5 : velocity 3 axes > max
	(*constant_vel)(0) = 0.5;
	(*constant_vel)(1) = 0.4;
	(*constant_vel)(2) = 0.6;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #5", isClose(translation_velocity.norm(), *maximum_velocity));

	// Step #6 : rotational velocity only
	constant_vel->setZero();
	(*constant_vel)(4) = 0.5;
	(*constant_vel)(4) = 0.4;
	(*constant_vel)(5) = 0.6;
	safety_controller.updateTCPVelocity();

	assert_msg("Step #6", tcp_velocity->isApprox(*total_velocity));

	return 0;
}
