#include <default_constraint.h>

using namespace RSCL;
using namespace Constraints;

/***		Constructor & destructor		***/
DefaultConstraint::DefaultConstraint(
	Constraints::ConstraintType type) :
	Constraint(type)
{
}

/***		Algorithm		***/
double DefaultConstraint::compute() {
	return 1.;
}
