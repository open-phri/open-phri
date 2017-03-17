#pragma once

#include <constraint.h>

namespace RSCL {

namespace Constraints {

class DefaultConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	DefaultConstraint(Constraints::ConstraintType type);

	virtual ~DefaultConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;
};

using DefaultConstraintPtr = std::shared_ptr<DefaultConstraint>;
using DefaultConstraintConstPtr = std::shared_ptr<const DefaultConstraint>;

} // namespace Constraints

} // namespace RSCL
