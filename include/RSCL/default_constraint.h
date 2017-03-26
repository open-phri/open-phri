#pragma once

#include <RSCL/constraint.h>

namespace RSCL {

class DefaultConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	DefaultConstraint(ConstraintType type);

	virtual ~DefaultConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;
};

using DefaultConstraintPtr = std::shared_ptr<DefaultConstraint>;
using DefaultConstraintConstPtr = std::shared_ptr<const DefaultConstraint>;

} // namespace RSCL
