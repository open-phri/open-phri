#include <RSCL/separation_distance_constraint.h>

#include <iostream>

using namespace RSCL;

SeparationDistanceConstraint::SeparationDistanceConstraint(
	ConstraintPtr constraint,
	InterpolatorPtr interpolator) :
	Constraint(constraint->getType())
{
	constraint_ = constraint;
	interpolator_ = interpolator;
	verbose_ = false;

	separation_distance_ = std::make_shared<double>();

	interpolator->setInput(separation_distance_);
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

double SeparationDistanceConstraint::closestObjectDistance() {
	Vector3d rob_pos;
	if(robot_position_) {
		rob_pos = robot_position_->block<3,1>(0,0);
	}
	else {
		rob_pos = Vector3d::Zero();
	}

	double min_dist = std::numeric_limits<double>::infinity();
	for(const auto& object : objects_) {
		Vector3d obj_rob_vec = object.second->block<3,1>(0,0) - rob_pos;

		min_dist = std::min(min_dist, obj_rob_vec.norm());
	}

	return min_dist;
}

void SeparationDistanceConstraint::setVerbose(bool on) {
	verbose_ = on;
}

bool SeparationDistanceConstraint::addObject(const std::string& name, Vector6dConstPtr object, bool force) {
	if((objects_.find(name) != objects_.end())and not force) {
		if(verbose_) {
			std::cerr << "In SeparationDistanceConstraint::addObject: an object called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
		}
		return false;
	}
	objects_[name] = object;
	return true;
}

bool SeparationDistanceConstraint::removeObject(const std::string& name) {
	auto elem = objects_.find(name);
	if(elem == objects_.end()) {
		if(verbose_) {
			std::cerr << "In SeparationDistanceConstraint::removeObject: no object called \"" << name << "\"" << std::endl;
		}
		return false;
	}
	objects_.erase(elem);
	return true;
}

Vector6dConstPtr SeparationDistanceConstraint::getObject(const std::string& name) {
	Vector6dConstPtr ptr;
	auto elem = objects_.find(name);
	if(elem != objects_.end()) {
		ptr = elem->second;
	}
	else if(verbose_) {
		std::cerr << "In SeparationDistanceConstraint::getObject: no object called \"" << name << "\"" << std::endl;
	}
	return ptr;
}
