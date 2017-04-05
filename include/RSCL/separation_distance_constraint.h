#pragma once

#include <RSCL/definitions.h>
#include <RSCL/constraint.h>
#include <RSCL/interpolator.h>
#include <map>

namespace RSCL {

class SeparationDistanceConstraint : public Constraint {
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

	/***       Configuration     ***/
	void setVerbose(bool on);

	bool addObject(const std::string& name, Vector6dConstPtr object, bool force = false);
	bool removeObject(const std::string& name);
	Vector6dConstPtr getObject(const std::string& name);

private:
	double closestObjectDistance();

	ConstraintPtr constraint_;
	InterpolatorPtr interpolator_;
	Vector6dConstPtr robot_position_;
	doublePtr separation_distance_;

	std::map<std::string, Vector6dConstPtr> objects_;

	bool verbose_;
};

using SeparationDistanceConstraintPtr = std::shared_ptr<SeparationDistanceConstraint>;
using SeparationDistanceConstraintConstPtr = std::shared_ptr<const SeparationDistanceConstraint>;

} // namespace RSCL
