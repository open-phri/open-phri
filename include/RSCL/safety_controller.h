#pragma once

#include <memory>
#include <map>

#include <definitions.h>
#include <constraint.h>
#include <force_generator.h>
#include <velocity_generator.h>

namespace RSCL {

class SafetyController {
public:
	SafetyController(Matrix6dConstPtr damping_matrix);

	~SafetyController() = default;

	void setVerbose(bool on);

	bool addConstraint(const std::string& name, ConstraintPtr constraint, bool force = false);
	bool addForceGenerator(const std::string& name, ForceGeneratorPtr generator, bool force = false);
	bool addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator, bool force = false);

	bool removeConstraint(const std::string& name);
	bool removeForceGenerator(const std::string& name);
	bool removeVelocityGenerator(const std::string& name);

	ConstraintPtr getConstraint(const std::string& name);
	ForceGeneratorPtr getForceGenerator(const std::string& name);
	VelocityGeneratorPtr getVelocityGenerator(const std::string& name);

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

	std::map<std::string, ConstraintPtr>           constraints_;
	std::map<std::string, ForceGeneratorPtr>       force_generators_;
	std::map<std::string, VelocityGeneratorPtr>    velocity_generators_;

	Vector6dPtr tcp_velocity_;
	Vector6dPtr total_velocity_;
	Vector6dPtr total_force_;
	Matrix6dConstPtr damping_matrix_;

	bool verbose_;
};

} // namespace RSCL
