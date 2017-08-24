#include <OpenPHRI/force_generators/stiffness_generator.h>
#include <iostream>

using namespace phri;

StiffnessGenerator::StiffnessGenerator(
	Matrix6dConstPtr stiffness,
	Vector6dConstPtr target_position,
	ReferenceFrame stiffness_frame,
	ReferenceFrame target_position_frame) :
	ForceGenerator(stiffness_frame),
	stiffness_(stiffness),
	target_position_(target_position),
	stiffness_frame_(stiffness_frame),
	target_position_frame_(target_position_frame)
{
}


Vector6d StiffnessGenerator::compute() {
	Vector6d error;

	if(stiffness_frame_ == ReferenceFrame::TCP) {
		if(target_position_frame_ == ReferenceFrame::TCP) {
			error = *target_position_;
		}
		else {
			error = robot_->spatialTransformationMatrix()->transpose() * (*target_position_ - *robot_->controlPointCurrentPose());
		}
	}
	else {
		if(target_position_frame_ == ReferenceFrame::TCP) {
			error = *robot_->spatialTransformationMatrix() * *target_position_;
		}
		else {
			error = *target_position_ - *robot_->controlPointCurrentPose();
		}
	}

	force_ = *stiffness_ * error;

	return ForceGenerator::compute();
}
