#pragma once

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

namespace Constraints {

class ForceConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	ForceConstraint(
		Vector6dPtr external_force,
		doublePtr maximum_force);
	ForceConstraint(
		Vector6dConstPtr external_force,
		doubleConstPtr maximum_force);

	virtual ~ForceConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	Vector6dConstPtr external_force_;
	doubleConstPtr maximum_force_;
};

using ForceConstraintPtr = std::shared_ptr<ForceConstraint>;
using ForceConstraintConstPtr = std::shared_ptr<const ForceConstraint>;

} // namespace Constraints

} // namespace RSCL
