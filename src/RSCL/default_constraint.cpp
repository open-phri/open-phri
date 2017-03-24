#include <default_constraint.h>

using namespace RSCL;


/***		Constructor & destructor		***/
DefaultConstraint::DefaultConstraint(
	ConstraintType type) :
	Constraint(type)
{
}

/***		Algorithm		***/
double DefaultConstraint::compute() {
	return 1.;
}
