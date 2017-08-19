#include <OpenPHRI/force_generators/mass_generator.h>

using namespace OpenPHRI;

MassGenerator::MassGenerator(
	Matrix6dConstPtr mass,
	Vector6dConstPtr target_acceleration,
	ReferenceFrame mass_frame,
	ReferenceFrame target_acceleration_frame) :
	ForceGenerator(mass_frame),
	mass_(mass),
	target_acceleration_(target_acceleration),
	mass_frame_(mass_frame),
	target_acceleration_frame_(target_acceleration_frame)
{
}


Vector6d MassGenerator::compute() {
	Vector6d error;

	if(mass_frame_ == ReferenceFrame::TCP) {
		if(target_acceleration_frame_ == ReferenceFrame::TCP) {
			error = *target_acceleration_;
		}
		else {
			error = robot_->spatialTransformationMatrix()->transpose() * (*target_acceleration_ - *robot_->controlPointCurrentAcceleration());
		}
		force_ = *mass_ * error;
	}
	else {
		if(target_acceleration_frame_ == ReferenceFrame::TCP) {
			error = *robot_->spatialTransformationMatrix() * *target_acceleration_;
		}
		else {
			error = *target_acceleration_ - *robot_->controlPointCurrentAcceleration();
		}
		force_ = *mass_ * error;
	}

	return ForceGenerator::compute();
}
