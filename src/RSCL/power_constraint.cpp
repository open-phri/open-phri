#include <power_constraint.h>

using namespace RSCL;

using namespace Eigen;

/***		Constructor & destructor		***/
PowerConstraint::PowerConstraint(
	Vector6dConstPtr total_velocity,
	Vector6dConstPtr external_force,
	doubleConstPtr maximum_power) :
	Constraint(ConstraintType::Minimum),
	total_velocity_(total_velocity),
	external_force_(external_force),
	maximum_power_(maximum_power)
{
	power_ = std::make_shared<double>(0);
}

/***		Algorithm		***/
double PowerConstraint::compute() {
	double constraint = 1.;
	const Vector3d& velocity = total_velocity_->block<3,1>(0,0);
	const Vector3d& force = external_force_->block<3,1>(0,0);
	double power = force.dot(velocity);
	*power_ = power;

	if(power < 0.) {
		constraint = std::abs(*maximum_power_ / power);
	}

	return constraint;
}

doubleConstPtr PowerConstraint::getPower() const {
	return power_;
}
