#include <RSCL/constraint.h>

using namespace RSCL;


Constraint::Constraint(ConstraintType type) : type_(type)
{

}

ConstraintType Constraint::getType() const {
	return type_;
}
