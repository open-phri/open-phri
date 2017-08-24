#include <OpenPHRI/constraints/joint_velocity_constraint.h>

using namespace phri;

using namespace Eigen;

/***		Constructor & destructor		***/
JointVelocityConstraint::JointVelocityConstraint(
	VectorXdConstPtr maximum_velocities) :
	maximum_velocities_(maximum_velocities)
{
}

/***		Algorithm		***/
double JointVelocityConstraint::compute() {
	double constraint = 1.;
	const auto& joint_vel = *robot_->jointTotalVelocity();
	const auto& max_joint_vel = *maximum_velocities_;

	for (size_t i = 0; i < joint_vel.size(); ++i) {
		constraint = std::min(constraint, max_joint_vel(i)/std::abs(joint_vel(i)));
	}

	return constraint;
}
