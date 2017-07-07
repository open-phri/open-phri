#include <RSCL/velocity_generators/velocity_generator.h>

using namespace RSCL;

Vector6d VelocityGenerator::operator()() {
	return compute();
}
