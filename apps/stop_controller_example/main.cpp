#include <iostream>

#include <RSCL.h>
#include <constraints.h>
#include <velocity_generators.h>
#include <force_generators.h>

using namespace std;
using namespace RSCL;

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());
	auto ext_force = make_shared<Vector6d>(Vector6d::Zero());
	auto activation_force_threshold = make_shared<double>(1.);
	auto deactivation_force_threshold = make_shared<double>(0.8);

	auto safety_controller = SafetyController(damping_matrix);
	auto tcp_velocity = safety_controller.getTCPVelocity();

	auto stop_constraint = make_shared<Constraints::StopConstraint>(ext_force, activation_force_threshold, deactivation_force_threshold);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	(*constant_vel)(0) = 1.;
	auto constant_velocity_generator = make_shared<ConstantVelocityGenerator>(constant_vel);
	auto constant_force_generator = make_shared<ConstantForceGenerator>(ext_force);

	safety_controller.addConstraint("stop constraint", stop_constraint);
	safety_controller.addVelocityGenerator("const vel", constant_velocity_generator);
	safety_controller.addForceGenerator("const force", constant_force_generator);

	for (size_t i = 0; i <= 10; ++i) {
		safety_controller.updateTCPVelocity();
		cout << "****************************************\n";
		cout << "ext_force: " << ext_force->transpose() << endl;
		cout << "tcp_velocity: " << tcp_velocity->transpose() << endl;
		cout << "tcp_velocity norm: " << tcp_velocity->norm() << endl;

		i < 5 ? ext_force->y() += 0.3 : ext_force->y() -= 0.3;
	}

	return 0;
}
