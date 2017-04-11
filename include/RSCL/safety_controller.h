#pragma once

#include <memory>
#include <map>

#include <RSCL/definitions.h>
#include <RSCL/fwd_decl.h>
#include <RSCL/object_collection.hpp>

namespace RSCL {

class SafetyController {
public:
	SafetyController(Matrix6dConstPtr damping_matrix);

	~SafetyController() = default;

	void setVerbose(bool on);

	bool addConstraint(const std::string& name, std::shared_ptr<Constraint> constraint, bool force = false);
	bool addForceGenerator(const std::string& name, std::shared_ptr<ForceGenerator> generator, bool force = false);
	bool addVelocityGenerator(const std::string& name, std::shared_ptr<VelocityGenerator> generator, bool force = false);

	template<typename T>
	typename std::enable_if<std::is_base_of<Constraint, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addConstraint(name, obj, force);
	}

	template<typename T>
	typename std::enable_if<std::is_base_of<ForceGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addForceGenerator(name, obj, force);
	}

	template<typename T>
	typename std::enable_if<std::is_base_of<VelocityGenerator, T>::value, bool>::type
	add(const std::string& name, std::shared_ptr<T> obj, bool force = false) {
		return addVelocityGenerator(name, obj, force);
	}

	bool removeConstraint(const std::string& name);
	bool removeForceGenerator(const std::string& name);
	bool removeVelocityGenerator(const std::string& name);

	std::shared_ptr<Constraint> getConstraint(const std::string& name);
	std::shared_ptr<ForceGenerator> getForceGenerator(const std::string& name);
	std::shared_ptr<VelocityGenerator> getVelocityGenerator(const std::string& name);

	void updateTCPVelocity();

	// Controller velocity output (in TCP frame)
	Vector6dConstPtr getTCPVelocity() const;
	// Sum of all the generated velocities (=tcp velocity if constraint=1). Updated during updateToolVelocity
	Vector6dConstPtr getTotalVelocity() const;
	// Sum of all the generated forces. Updated during updateToolVelocity
	Vector6dConstPtr getTotalForce() const;


private:
	double computeConstraintValue() const;
	Vector6d computeForceSum() const;
	Vector6d computeVelocitySum() const;

	ObjectCollection<std::shared_ptr<Constraint>>           constraints_;
	ObjectCollection<std::shared_ptr<ForceGenerator>>       force_generators_;
	ObjectCollection<std::shared_ptr<VelocityGenerator>>    velocity_generators_;

	Vector6dPtr tcp_velocity_;
	Vector6dPtr total_velocity_;
	Vector6dPtr total_force_;
	Matrix6dConstPtr damping_matrix_;
};

} // namespace RSCL
