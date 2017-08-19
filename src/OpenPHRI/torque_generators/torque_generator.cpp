#include <OpenPHRI/torque_generators/torque_generator.h>

using namespace OpenPHRI;

VectorXd TorqueGenerator::operator()() {
	return compute();
}
