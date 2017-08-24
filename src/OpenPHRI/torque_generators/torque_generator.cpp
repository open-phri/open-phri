#include <OpenPHRI/torque_generators/torque_generator.h>

using namespace phri;

VectorXd TorqueGenerator::operator()() {
	return compute();
}
