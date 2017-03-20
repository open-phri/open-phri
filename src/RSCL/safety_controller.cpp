#include <safety_controller.h>
#include <default_constraint.h>

#include <limits>
#include <iostream>

using namespace RSCL;

SafetyController::SafetyController(
	Matrix6dConstPtr damping_matrix) :
	damping_matrix_(damping_matrix),
	verbose_(false)
{
	tcp_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_force_ = std::make_shared<Vector6d>(Vector6d::Zero());

	addConstraint("default min", std::make_shared<Constraints::DefaultConstraint>(Constraints::ConstraintType::Minimum));
	addConstraint("default mult", std::make_shared<Constraints::DefaultConstraint>(Constraints::ConstraintType::Multiplicative));
}

void SafetyController::setVerbose(bool on) {
	verbose_ = on;
}

bool SafetyController::addConstraint(const std::string& name, ConstraintPtr constraint, bool force) {
	if((constraints_.find(name) != constraints_.end())and not force) {
		if(verbose_) {
			std::cerr << "In SafetyController::addConstraint: a constraint called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
		}
		return false;
	}
	constraints_[name] = constraint;
	return true;
}

bool SafetyController::addForceGenerator(const std::string& name, ForceGeneratorPtr generator, bool force) {
	if((force_generators_.find(name) != force_generators_.end())and not force) {
		if(verbose_) {
			std::cerr << "In SafetyController::addForceGenerator: a generator called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
		}
		return false;
	}
	force_generators_[name] = generator;
	return true;
}

bool SafetyController::addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator, bool force) {
	if((velocity_generators_.find(name) != velocity_generators_.end())and not force) {
		if(verbose_) {
			std::cerr << "In SafetyController::addVelocityGenerator: a generator called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
		}
		return false;
	}
	velocity_generators_[name] = generator;
	return true;
}

bool SafetyController::removeConstraint(const std::string& name) {
	auto elem = constraints_.find(name);
	if(elem == constraints_.end()) {
		if(verbose_) {
			std::cerr << "In SafetyController::removeConstraint: no constraint called \"" << name << "\"" << std::endl;
		}
		return false;
	}
	constraints_.erase(elem);
	return true;
}

bool SafetyController::removeForceGenerator(const std::string& name) {
	auto elem = force_generators_.find(name);
	if(elem == force_generators_.end()) {
		if(verbose_) {
			std::cerr << "In SafetyController::removeForceGenerator: no generator called \"" << name << "\"" << std::endl;
		}
		return false;
	}
	force_generators_.erase(elem);
	return true;
}

bool SafetyController::removeVelocityGenerator(const std::string& name) {
	auto elem = velocity_generators_.find(name);
	if(elem == velocity_generators_.end()) {
		if(verbose_) {
			std::cerr << "In SafetyController::removeVelocityGenerator: no generator called \"" << name << "\"" << std::endl;
		}
		return false;
	}
	velocity_generators_.erase(elem);
	return true;
}

ConstraintPtr SafetyController::getConstraint(const std::string& name) {
	ConstraintPtr ptr;
	auto elem = constraints_.find(name);
	if(elem != constraints_.end()) {
		ptr = elem->second;
	}
	else if(verbose_) {
		std::cerr << "In SafetyController::getConstraint: no constraint called \"" << name << "\"" << std::endl;
	}
	return ptr;
}

ForceGeneratorPtr SafetyController::getForceGenerator(const std::string& name) {
	ForceGeneratorPtr ptr;
	auto elem = force_generators_.find(name);
	if(elem != force_generators_.end()) {
		ptr = elem->second;
	}
	else if(verbose_) {
		std::cerr << "In SafetyController::getForceGenerator: no generator called \"" << name << "\"" << std::endl;
	}
	return ptr;
}

VelocityGeneratorPtr SafetyController::getVelocityGenerator(const std::string& name) {
	VelocityGeneratorPtr ptr;
	auto elem = velocity_generators_.find(name);
	if(elem != velocity_generators_.end()) {
		ptr = elem->second;
	}
	else if(verbose_) {
		std::cerr << "In SafetyController::getVelocityGenerator: no generator called \"" << name << "\"" << std::endl;
	}
	return ptr;
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

double SafetyController::computeConstraintValue() const {
	double min_value = std::numeric_limits<double>::infinity();
	double mult_value = 1.;

	for(const auto& constraint : constraints_) {
		if(constraint.second->getType() == Constraints::ConstraintType::Minimum) {
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
