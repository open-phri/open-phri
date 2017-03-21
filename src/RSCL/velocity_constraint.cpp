#include <velocity_constraint.h>

using namespace RSCL;
using namespace Constraints;
using namespace Eigen;

/***		Constructor & destructor		***/
VelocityConstraint::VelocityConstraint(
	Vector6dPtr total_velocity,
	doublePtr maximum_velocity) :
	Constraint(ConstraintType::Minimum),
	total_velocity_(total_velocity),
	maximum_velocity_(maximum_velocity)
{

}

VelocityConstraint::VelocityConstraint(
	Vector6dConstPtr total_velocity,
	doubleConstPtr maximum_velocity) :
	Constraint(ConstraintType::Minimum),
	total_velocity_(total_velocity),
	maximum_velocity_(maximum_velocity)
{

}

/***		Algorithm		***/
double VelocityConstraint::compute() {
	double constraint = 1.;
	double v_norm = total_velocity_->block<3,1>(0,0).norm();

	if(v_norm > 0.) {
		constraint = std::abs(*maximum_velocity_) / total_velocity_->norm();
	}

	return constraint;
}
