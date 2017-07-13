#include <RSCL/force_generators/force_generator.h>

using namespace RSCL;

ForceGenerator::ForceGenerator(ReferenceFrame frame) {
	frame_ = frame;
}

Vector6d ForceGenerator::compute() {
	return transform(force_);
}

Vector6d ForceGenerator::transform(Vector6d force) {
	if(frame_ != ReferenceFrame::TCP) {
		return robot_->spatialTransformationMatrix()->transpose() * force;
	}
	return force;
}

Vector6d ForceGenerator::operator()() {
	return compute();
}
