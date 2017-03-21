#pragma once

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

namespace Constraints {

class VelocityConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
	VelocityConstraint(
		Vector6dPtr total_velocity,
		doublePtr maximum_velocity);
	VelocityConstraint(
		Vector6dConstPtr total_velocity,
		doubleConstPtr maximum_velocity);

	virtual ~VelocityConstraint() = default;

	/***		Algorithm		***/
	virtual double compute() override;

private:
	Vector6dConstPtr total_velocity_;
	doubleConstPtr maximum_velocity_;
};

using VelocityConstraintPtr = std::shared_ptr<VelocityConstraint>;
using VelocityConstraintConstPtr = std::shared_ptr<const VelocityConstraint>;

} // namespace Constraints

} // namespace RSCL
