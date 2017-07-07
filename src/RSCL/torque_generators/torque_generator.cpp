#include <RSCL/torque_generators/torque_generator.h>

using namespace RSCL;

VectorXd TorqueGenerator::operator()() {
	return compute();
}
