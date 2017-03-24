#include <force_constraint.h>
#include <limits>

using namespace RSCL;

using namespace Eigen;

/***		Constructor & destructor		***/
ForceConstraint::ForceConstraint(
	Vector6dConstPtr external_force,
	doubleConstPtr maximum_force) :
	Constraint(ConstraintType::Minimum),
	external_force_(external_force),
	maximum_force_(maximum_force)
{

}

/***		Algorithm		***/
double ForceConstraint::compute() {
	double constraint = 1.;
	double f_norm = external_force_->block<3,1>(0,0).norm();

	// TODO can't work like this, need to find a proper solution to move away from the external force
	if(f_norm > *maximum_force_) {
		constraint = std::numeric_limits<double>::infinity();
	}

	return constraint;
}
