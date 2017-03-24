#pragma once

#include <definitions.h>
#include <constraint.h>

namespace RSCL {

class VelocityConstraint : public Constraint {
public:
	/***		Constructor & destructor		***/
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

} // namespace RSCL
