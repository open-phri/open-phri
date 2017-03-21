#include <constant_velocity_generator.h>

using namespace RSCL;

ConstantVelocityGenerator::ConstantVelocityGenerator(Vector6dPtr velocity) :
	velocity_(velocity)
{
}

ConstantVelocityGenerator::ConstantVelocityGenerator(Vector6dConstPtr velocity) :
	velocity_(velocity)
{
}

Vector6d ConstantVelocityGenerator::compute() {
	return *velocity_;
}
