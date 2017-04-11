#include <RSCL/stiffness_generator.h>

using namespace RSCL;

StiffnessGenerator::StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position) :
	stiffness_(stiffness),
	target_position_(target_position)
{
	robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

StiffnessGenerator::StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position, Vector6dConstPtr robot_position) :
	StiffnessGenerator(stiffness, target_position)
{
	robot_position_ = robot_position;
}

Vector6d StiffnessGenerator::compute() {
	return *stiffness_ * (*target_position_ - *robot_position_);
}
