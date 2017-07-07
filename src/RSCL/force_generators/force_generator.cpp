#include <RSCL/force_generators/force_generator.h>

using namespace RSCL;

Vector6d ForceGenerator::operator()() {
	return compute();
}
