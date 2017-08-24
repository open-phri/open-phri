#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>

using namespace phri;

VectorXd JointVelocityGenerator::operator()() {
	return compute();
}
