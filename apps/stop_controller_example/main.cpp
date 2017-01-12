#include <iostream>
#include <adaptive_damping.h>
#include <stop_controller.h>

using namespace std;
using namespace ADamp;

int main(int argc, char const *argv[]) {
	auto V_b_t = make_shared<Matrix6d>();
	auto damping = make_shared<Matrix6d>();
	auto ref_vel = make_shared<Vector6d>();
	auto force = make_shared<Vector6d>();
	auto ext_force = make_shared<Vector6d>();
	auto force_threshold = make_shared<double>();
	auto force_threshold_hysteresis = make_shared<double>();

	StopController controller(
		V_b_t,
		damping,
		ref_vel,
		force,
		ext_force,
		force_threshold,
		force_threshold_hysteresis);

	V_b_t->setIdentity();
	damping->setIdentity();
	damping->setIdentity() *= 100.;
	ref_vel->setConstant(1.);
	force->setZero();
	force->x() = 10;
	ext_force->setZero();
	*force_threshold = 1.;              // activation level
	*force_threshold_hysteresis = 0.2;  // deactivation hysteresis


	for (size_t i = 0; i < 10; ++i) {
		if(i < 5) {
			ext_force->y() += 0.3;
		}
		else {
			ext_force->y() -= 0.3;
		}
		auto tool_vel = controller.computeVelocity();
		cout << "****************************************\n";
		cout << "ext_force: " << ext_force->transpose() << endl;
		cout << "tool_vel: " << tool_vel.transpose() << endl;
	}

	return 0;
}
