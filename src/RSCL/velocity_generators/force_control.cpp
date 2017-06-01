#include <RSCL/velocity_generators/force_control.h>

using namespace RSCL;

ForceControl::ForceControl(
	Vector6dConstPtr external_force,
	Vector6dConstPtr external_force_target,
	double sample_time,
	Vector6dConstPtr p_gain,
	Vector6dConstPtr d_gain,
	Vector6dConstPtr selection) :
	external_force_(external_force),
	external_force_target_(external_force_target),
	sample_time_(sample_time),
	p_gain_(p_gain),
	d_gain_(d_gain),
	selection_(selection)
{
	prev_error_.setZero();
}

Vector6d ForceControl::compute() {
	Vector6d error = *external_force_target_ - *external_force_;
	applySelection(error);
	Vector6d cmd = p_gain_->cwiseProduct(error) + d_gain_->cwiseProduct((error - prev_error_)/sample_time_);
	prev_error_ = error;
	return cmd;
}

void ForceControl::applySelection(Vector6d& vec) {
	const auto& sel = *selection_;
	for (size_t i = 0; i < 6; ++i) {
		if(sel(i) == 0.) {
			vec(i) = 0.;
		}
	}
}
