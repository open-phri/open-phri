#include <RSCL/joint_velocity_generators/joint_velocity_generator.h>

using namespace RSCL;

VectorXd JointVelocityGenerator::operator()() {
	return compute();
}
