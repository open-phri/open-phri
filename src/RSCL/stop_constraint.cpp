#include <stop_constraint.h>
#include <iostream>

using namespace RSCL;
using namespace Constraints;
using namespace Eigen;

/***		Constructor & destructor		***/
StopConstraint::StopConstraint(
	Vector6dConstPtr external_force,
	doubleConstPtr activation_force_threshold,
	doubleConstPtr deactivation_force_threshold) :
	Constraint(ConstraintType::Multiplicative),
	external_force_(external_force),
	activation_force_threshold_(activation_force_threshold),
	deactivation_force_threshold_(deactivation_force_threshold),
	previous_constraint_value_(1.)
{

}

/***		Algorithm		***/
double StopConstraint::compute() {
	double constraint = previous_constraint_value_;
	double ft_act = *activation_force_threshold_;
	double ft_deact = *deactivation_force_threshold_;
	double f_norm = external_force_->block<3,1>(0,0).norm();

	if(f_norm >= ft_act) {
		constraint = 0.;
	}
	else if(f_norm <= ft_deact) {
		constraint = 1.;
	}

	previous_constraint_value_ = constraint;

	return constraint;
}
