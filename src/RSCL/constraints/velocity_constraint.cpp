#include <RSCL/constraints/velocity_constraint.h>

using namespace RSCL;

using namespace Eigen;

/***		Constructor & destructor		***/
VelocityConstraint::VelocityConstraint(
	doubleConstPtr maximum_velocity) :
	Constraint(ConstraintType::Minimum),
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
