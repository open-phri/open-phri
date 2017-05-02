#include <RSCL/utilities/interpolator.h>

using namespace RSCL;

doubleConstPtr Interpolator::getOutput() const {
	return output_;
}

void Interpolator::setInput(doubleConstPtr input) {
	input_ = input;
}
