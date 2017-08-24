#include <OpenPHRI/utilities/interpolator.h>

using namespace phri;

doubleConstPtr Interpolator::getOutput() const {
	return output_;
}

void Interpolator::setInput(doubleConstPtr input) {
	input_ = input;
}
