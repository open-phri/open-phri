#include <RSCL/force_generators/stiffness_generator.h>
#include <iostream>

using namespace RSCL;

StiffnessGenerator::StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position) :
	stiffness_(stiffness),
	target_position_(target_position)
{
	robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

StiffnessGenerator::StiffnessGenerator(
	Matrix6dConstPtr stiffness,
	Vector6dConstPtr target_position,
	Vector6dConstPtr robot_position,
	Matrix6dConstPtr spatial_transformation,
	bool do_transpose,
	bool stiffness_in_tcp_frame) :
	StiffnessGenerator(stiffness, target_position)
{
	robot_position_ = robot_position;
	spatial_transformation_ = spatial_transformation;
	do_transpose_ = do_transpose;
	stiffness_in_tcp_frame_ = stiffness_in_tcp_frame;
}

Vector6d StiffnessGenerator::compute() {
	Vector6d error = *target_position_ - *robot_position_;
	if(static_cast<bool>(spatial_transformation_)) {
		if(stiffness_in_tcp_frame_) {
			if(do_transpose_) {
				return *stiffness_ * spatial_transformation_->transpose() * error;
			}
			return *stiffness_ * *spatial_transformation_ * error;
		}
		else {
			if(do_transpose_) {
				return spatial_transformation_->transpose() * *stiffness_ * error;
			}
			return *spatial_transformation_ * *stiffness_ * error;
		}
	}
	return *stiffness_ * error;
}
