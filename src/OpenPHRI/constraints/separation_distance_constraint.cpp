#include <OpenPHRI/constraints/separation_distance_constraint.h>

using namespace OpenPHRI;

SeparationDistanceConstraint::SeparationDistanceConstraint(
	ConstraintPtr constraint,
	InterpolatorPtr interpolator)
{
	constraint_ = constraint;
	interpolator_ = interpolator;

	separation_distance_ = std::make_shared<double>();

	interpolator->setInput(separation_distance_);

	robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

SeparationDistanceConstraint::SeparationDistanceConstraint(
	ConstraintPtr constraint,
	InterpolatorPtr interpolator,
	Vector6dConstPtr robot_position) :
	SeparationDistanceConstraint(constraint, interpolator)
{
	robot_position_ = robot_position;
}

double SeparationDistanceConstraint::compute() {
	*separation_distance_ = closestObjectDistance();
	interpolator_->compute();
	return constraint_->compute();
}

doubleConstPtr SeparationDistanceConstraint::getSeparationDistance() const {
	return separation_distance_;
}

void SeparationDistanceConstraint::setRobot(RobotConstPtr robot) {
	constraint_->setRobot(robot);
	Constraint::setRobot(robot);
}

double SeparationDistanceConstraint::closestObjectDistance() {
	const Vector3d& rob_pos = robot_position_->block<3,1>(0,0);

	double min_dist = std::numeric_limits<double>::infinity();
	for(const auto& item : items_) {
		Vector3d obj_rob_vec = item.second->block<3,1>(0,0) - rob_pos;

		min_dist = std::min(min_dist, obj_rob_vec.norm());
	}

	return min_dist;
}
