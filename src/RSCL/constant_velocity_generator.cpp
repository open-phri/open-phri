#include <constant_force_generator.h>

using namespace RSCL;

ConstantForceGenerator::ConstantForceGenerator(Vector6dPtr force) :
	force_(force)
{
}

ConstantForceGenerator::ConstantForceGenerator(Vector6dConstPtr force) :
	force_(force)
{
}

Vector6d ConstantForceGenerator::compute() {
	return *force_;
}
