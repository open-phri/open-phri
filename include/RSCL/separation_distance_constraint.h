#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraint.h>
#include <RSCL/interpolator.h>
#include <RSCL/object_collection.hpp>
#include <map>

namespace RSCL {

class SeparationDistanceConstraint : public Constraint, public ObjectCollection<Vector6dConstPtr> {
public:
	/***		Constructor & destructor		***/
	SeparationDistanceConstraint(
		ConstraintPtr constraint,
		InterpolatorPtr interpolator);

	SeparationDistanceConstraint(
		ConstraintPtr constraint,
		InterpolatorPtr interpolator,
		Vector6dConstPtr robot_position);

	virtual ~SeparationDistanceConstraint() = default;

	/***		Algorithm        ***/
	virtual double compute() override;

private:
	double closestObjectDistance();

	ConstraintPtr constraint_;
	InterpolatorPtr interpolator_;
	Vector6dConstPtr robot_position_;
	doublePtr separation_distance_;
};

using SeparationDistanceConstraintPtr = std::shared_ptr<SeparationDistanceConstraint>;
using SeparationDistanceConstraintConstPtr = std::shared_ptr<const SeparationDistanceConstraint>;

} // namespace RSCL
