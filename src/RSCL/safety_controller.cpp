#include <RSCL/safety_controller.h>

#include <RSCL/constraints/default_constraint.h>
#include <RSCL/force_generators/force_generator.h>
#include <RSCL/velocity_generators/velocity_generator.h>

#include <limits>
#include <iostream>

using namespace RSCL;

SafetyController::SafetyController(
	Matrix6dConstPtr damping_matrix) :
	damping_matrix_(damping_matrix)
{
	tcp_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_force_ = std::make_shared<Vector6d>(Vector6d::Zero());

	addConstraint("default min", std::make_shared<DefaultConstraint>(ConstraintType::Minimum));
	addConstraint("default mult", std::make_shared<DefaultConstraint>(ConstraintType::Multiplicative));
}

void SafetyController::setVerbose(bool on) {
	constraints_.setVerbose         (on, "RSCL::SafetyController", "Constraint");
	force_generators_.setVerbose    (on, "RSCL::SafetyController", "ForceGenerator");
	velocity_generators_.setVerbose (on, "RSCL::SafetyController", "VelocityGenerator");
}

bool SafetyController::addConstraint(const std::string& name, ConstraintPtr constraint, bool force) {
	return constraints_.add(name, constraint, force);
}

bool SafetyController::addForceGenerator(const std::string& name, ForceGeneratorPtr generator, bool force) {
	return force_generators_.add(name, generator, force);
}

bool SafetyController::addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator, bool force) {
	return velocity_generators_.add(name, generator, force);
}

bool SafetyController::removeConstraint(const std::string& name) {
	return constraints_.remove(name);
}

bool SafetyController::removeForceGenerator(const std::string& name) {
	return force_generators_.remove(name);
}

bool SafetyController::removeVelocityGenerator(const std::string& name) {
	return velocity_generators_.remove(name);
}

ConstraintPtr SafetyController::getConstraint(const std::string& name) {
	return constraints_.get(name);
}

ForceGeneratorPtr SafetyController::getForceGenerator(const std::string& name) {
	return force_generators_.get(name);
}

VelocityGeneratorPtr SafetyController::getVelocityGenerator(const std::string& name) {
	return velocity_generators_.get(name);
}


void SafetyController::updateTCPVelocity() {
	Vector6d force_sum = computeForceSum();
	Vector6d velocity_sum = computeVelocitySum();

	*total_velocity_ = damping_matrix_->inverse()*force_sum + velocity_sum;
	*total_force_ = force_sum;

	double constraint_value = computeConstraintValue();

	*tcp_velocity_ = constraint_value * (*total_velocity_);
}

Vector6dConstPtr SafetyController::getTCPVelocity() const {
	return tcp_velocity_;
}

Vector6dConstPtr SafetyController::getTotalVelocity() const {
	return total_velocity_;
}

Vector6dConstPtr SafetyController::getTotalForce() const {
	return total_force_;
}

double SafetyController::computeConstraintValue() const {
	double min_value = std::numeric_limits<double>::infinity();
	double mult_value = 1.;

	for(const auto& constraint : constraints_) {
		if(constraint.second->getType() == ConstraintType::Minimum) {
			min_value = std::min(min_value, constraint.second->compute());
		}
		else {
			mult_value *= constraint.second->compute();
		}
	}

	if(min_value == std::numeric_limits<double>::infinity()) {
		min_value = 1.;
	}

	return mult_value * min_value;
}

Vector6d SafetyController::computeForceSum() const {
	Vector6d sum = Vector6d::Zero();

	for(const auto& force_generator : force_generators_) {
		sum += force_generator.second->compute();
	}

	return sum;
}

Vector6d SafetyController::computeVelocitySum() const {
	Vector6d sum = Vector6d::Zero();

	for(const auto& velocity_generator : velocity_generators_) {
		sum += velocity_generator.second->compute();
	}

	return sum;
}
