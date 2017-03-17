#include <safety_controller.h>

#include <default_constraint.h>

using namespace RSCL;

SafetyController::SafetyController(
	Matrix6dConstPtr damping_matrix) :
	damping_matrix_(damping_matrix)
{
	tcp_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_velocity_ = std::make_shared<Vector6d>(Vector6d::Zero());
	total_force_ = std::make_shared<Vector6d>(Vector6d::Zero());

	addConstraint("default", std::make_shared<Constraints::DefaultConstraint>(Constraints::ConstraintType::Minimum));
	addConstraint("default", std::make_shared<Constraints::DefaultConstraint>(Constraints::ConstraintType::Multiplicative));
}

void SafetyController::addConstraint(const std::string& name, ConstraintPtr constraint) {
	if(constraint->getType() == Constraints::ConstraintType::Multiplicative) {
		multiplicative_constraints_[name] = constraint;
	}
	else {
		minimum_constraints_[name] = constraint;
	}
}

void SafetyController::addForceGenerator(const std::string& name, ForceGeneratorPtr generator) {
	force_generators_[name] = generator;
}

void SafetyController::addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator) {
	velocity_generators_[name] = generator;
}

bool SafetyController::removeConstraint(const std::string& name) {
	auto elem = multiplicative_constraints_.find(name);
	if(elem == multiplicative_constraints_.end()) {
		elem = minimum_constraints_.find(name);
		if(elem == minimum_constraints_.end()) {
			return false;
		}
		minimum_constraints_.erase(elem);
	}
	else {
		multiplicative_constraints_.erase(elem);
	}
	return true;
}

bool SafetyController::removeForceGenerator(const std::string& name) {
	auto elem = force_generators_.find(name);
	if(elem == force_generators_.end()) {
		return false;
	}
	force_generators_.erase(elem);
	return true;
}

bool SafetyController::removeVelocityGenerator(const std::string& name) {
	auto elem = velocity_generators_.find(name);
	if(elem == velocity_generators_.end()) {
		return false;
	}
	velocity_generators_.erase(elem);
	return true;
}

ConstraintPtr SafetyController::getConstraint(const std::string& name) {
	ConstraintPtr ptr;
	auto elem = multiplicative_constraints_.find(name);
	if(elem == multiplicative_constraints_.end()) {
		elem = minimum_constraints_.find(name);
		if(elem != minimum_constraints_.end()) {
			ptr = elem->second;
		}
	}
	else {
		ptr = elem->second;
	}
	return ptr;
}

ForceGeneratorPtr SafetyController::getForceGenerator(const std::string& name) {
	ForceGeneratorPtr ptr;
	auto elem = force_generators_.find(name);
	if(elem != force_generators_.end()) {
		ptr = elem->second;
	}
	return ptr;
}

VelocityGeneratorPtr SafetyController::getVelocityGenerator(const std::string& name) {
	VelocityGeneratorPtr ptr;
	auto elem = velocity_generators_.find(name);
	if(elem != velocity_generators_.end()) {
		ptr = elem->second;
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
	double min_value;
	double mult_value = 1.;

	if(minimum_constraints_.empty()) {
		min_value = 1.;
	}
	else {
		// Get the first value
		auto min_it = minimum_constraints_.begin();
		min_value = min_it->second->compute();

		// Search for the minimum
		while(++min_it != minimum_constraints_.end()) {
			min_value = std::min(min_value, min_it->second->compute());
		}
	}

	for(const auto& constraint : multiplicative_constraints_) {
		mult_value *= constraint.second->compute();
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
