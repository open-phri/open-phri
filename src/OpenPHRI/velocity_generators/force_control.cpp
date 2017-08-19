#include <OpenPHRI/velocity_generators/force_control.h>
#include <iostream>

using namespace OpenPHRI;

ForceControl::ForceControl(
	Vector6dConstPtr external_force_target,
	double sample_time,
	Vector6dConstPtr p_gain,
	Vector6dConstPtr d_gain,
	Vector6dConstPtr selection,
	ReferenceFrame frame) :
	VelocityGenerator(frame),
	external_force_target_(external_force_target),
	sample_time_(sample_time),
	p_gain_(p_gain),
	d_gain_(d_gain),
	selection_(selection),
	frame_(frame),
	filter_coeff_(1.)
{
	prev_error_.setZero();
}

void ForceControl::configureFilter(double sample_time, double time_constant) {
	assert(sample_time > 0);
	assert(time_constant > 0);

	if(not (time_constant > 5.*sample_time)) {
		std::cout << "ForceControl::configureFilter: the time constant (" << time_constant << ") for the low pass filter should be at least five times greater than the sample time (" << sample_time << ") to have a correct behavior" << std::endl;
	}

	filter_coeff_ = (sample_time/(time_constant+sample_time));

}

Vector6d ForceControl::compute() {
	Vector6d error;
	// TODO Find how to chose between the targeted force applied to the environment or applied to the robot
	if(frame_ == ReferenceFrame::TCP) {
		error = *external_force_target_ + *robot_->controlPointExternalForce();
	}
	else {
		error = *external_force_target_ + *robot_->spatialTransformationMatrix() * *robot_->controlPointExternalForce();

	}
	applySelection(error);

	Vector6d cmd = p_gain_->cwiseProduct(error);

	error = filter_coeff_*error + (1.-filter_coeff_) *prev_error_;
	cmd += d_gain_->cwiseProduct((error - prev_error_)/sample_time_);
	prev_error_ = error;

	return cmd;
}

void ForceControl::applySelection(Vector6d& vec) {
	const auto& sel = *selection_;
	for (size_t i = 0; i < 6; ++i) {
		if(std::abs(sel(i)) < 1e-12) {
			vec(i) = 0.;
		}
	}
}
