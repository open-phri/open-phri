#include <constraint.h>

using namespace RSCL;
using namespace Constraints;

Constraint::Constraint(ConstraintType type) : type_(type)
{

}

ConstraintType Constraint::getType() {
	return type_;
}
