#include <OpenPHRI/constraints/force_constraint.h>
#include <limits>

using namespace phri;

using namespace Eigen;

class ForceLimitationVelocityGenerator : public VelocityGenerator {
public:
	ForceLimitationVelocityGenerator(Vector6dConstPtr external_force, doubleConstPtr maximum_force) :
		VelocityGenerator(ReferenceFrame::TCP),
		external_force_(external_force),
		maximum_force_(maximum_force)
	{

	}

	virtual Vector6d compute() override {
		velocity_ = Vector6d::Zero();
		double f_norm = external_force_->norm();

		if(f_norm > *maximum_force_) {
			velocity_ = *external_force_ * 1e12; // Just something huge
		}

		return VelocityGenerator::compute();
	}

private:
	Vector6dConstPtr external_force_;
	doubleConstPtr maximum_force_;
};

/***		Constructor & destructor		***/
ForceConstraint::ForceConstraint(
	VelocityConstraintPtr constraint,
	doubleConstPtr maximum_force) :
	constraint_(constraint),
	maximum_force_(maximum_force)
{
	velocity_generator_ = std::make_shared<ForceLimitationVelocityGenerator>(
		robot_->controlPointExternalForce(),
		maximum_force_);
}

/***		Algorithm		***/
double ForceConstraint::compute() {
	return constraint_->compute();
}

VelocityGeneratorPtr ForceConstraint::getVelocityGenerator() const {
	return velocity_generator_;
}
