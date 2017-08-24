#include <OpenPHRI/velocity_generators/velocity_generator.h>

using namespace phri;

VelocityGenerator::VelocityGenerator(ReferenceFrame frame) {
	frame_ = frame;
}

Vector6d VelocityGenerator::compute() {
	return transform(velocity_);
}

Vector6d VelocityGenerator::transform(Vector6d force) {
	if(frame_ != ReferenceFrame::TCP) {
		return robot_->spatialTransformationMatrix()->transpose() * force;
	}
	return force;
}

Vector6d VelocityGenerator::operator()() {
	return compute();
}
