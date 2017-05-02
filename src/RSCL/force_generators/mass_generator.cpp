#include <RSCL/force_generators/mass_generator.h>

using namespace RSCL;

MassGenerator::MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration) :
	mass_(mass),
	target_acceleration_(target_acceleration)
{
	robot_acceleration_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

MassGenerator::MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration, Vector6dConstPtr robot_acceleration) :
	MassGenerator(mass, target_acceleration)
{
	robot_acceleration_ = robot_acceleration;
}

Vector6d MassGenerator::compute() {
	return *mass_ * (*target_acceleration_ - *robot_acceleration_);
}
