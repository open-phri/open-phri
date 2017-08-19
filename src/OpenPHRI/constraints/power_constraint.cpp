#include <OpenPHRI/constraints/power_constraint.h>

using namespace OpenPHRI;

using namespace Eigen;

/***		Constructor & destructor		***/
PowerConstraint::PowerConstraint(
	doubleConstPtr maximum_power) :
	maximum_power_(maximum_power)
{
	power_ = std::make_shared<double>(0);
}

/***		Algorithm		***/
double PowerConstraint::compute() {
	double constraint = 1.;
	const Vector3d& velocity = robot_->controlPointTotalVelocity()->block<3,1>(0,0);
	const Vector3d& force = robot_->controlPointExternalForce()->block<3,1>(0,0);
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
