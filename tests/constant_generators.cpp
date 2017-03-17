#include <safety_controller.h>
#include <velocity_generators.h>
#include <force_generators.h>

using namespace RSCL;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
	return std::abs(v1-v2) < eps;
}

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());
	*damping_matrix *= 10.;

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_force = make_shared<Vector6d>(Vector6d::Zero());

	auto constant_velocity_generator = make_shared<ConstantVelocityGenerator>(constant_vel);
	auto constant_force_generator = make_shared<ConstantForceGenerator>(constant_force);

	auto safety_controller = SafetyController(damping_matrix);
	auto tcp_velocity = safety_controller.getTCPVelocity();

	safety_controller.addVelocityGenerator("const vel", constant_velocity_generator);
	safety_controller.addForceGenerator("const force", constant_force_generator);

	// Step #1 : no velocity, no force
	safety_controller.updateTCPVelocity();

	assert(("Step #1", tcp_velocity->isZero()));

	// Step #2 : velocity 1 axis, no force
	(*constant_vel)(0) = 0.5;

	safety_controller.updateTCPVelocity();

	assert(("Step #2", isClose(tcp_velocity->norm(), 0.5)));

	// Step #3 : velocity 2 axes, no force
	(*constant_vel)(0) = 1.;
	(*constant_vel)(3) = 1.;

	safety_controller.updateTCPVelocity();

	assert(("Step #3", isClose(tcp_velocity->norm(), std::sqrt(2.))));

	// Step #4 : no velocity, force 1 axis
	constant_vel->setZero();
	(*constant_force)(2) = 20.;

	safety_controller.updateTCPVelocity();

	assert(("Step #4", isClose(tcp_velocity->norm(), 2.)));

	// Step #5 : no velocity, force 2 axes
	(*constant_force)(2) = 10.;
	(*constant_force)(5) = 10.;

	safety_controller.updateTCPVelocity();

	assert(("Step #5", isClose(tcp_velocity->norm(), std::sqrt(2.))));

	// Step #6 : velocity 3 axes, force 3 axes, separate axes
	constant_vel->setZero();
	constant_force->setZero();
	(*constant_vel)(0) = 1.;
	(*constant_vel)(2) = 1.;
	(*constant_vel)(4) = 1.;
	(*constant_force)(1) = 10.;
	(*constant_force)(3) = 10.;
	(*constant_force)(5) = 10.;

	safety_controller.updateTCPVelocity();

	assert(("Step #6", isClose(tcp_velocity->norm(), std::sqrt(6.))));

	// Step #7 : velocity 3 axes, force 3 axes, mixed axes
	constant_vel->setZero();
	constant_force->setZero();
	(*constant_vel)(0) = 1.;
	(*constant_vel)(3) = 1.;
	(*constant_vel)(4) = 1.;
	(*constant_force)(1) = 10.;
	(*constant_force)(3) = 10.;
	(*constant_force)(5) = 10.;

	safety_controller.updateTCPVelocity();

	assert(("Step #7", isClose(tcp_velocity->norm(), std::sqrt(8.))));

	return 0;
}
