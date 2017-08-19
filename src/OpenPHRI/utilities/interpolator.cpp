#include <OpenPHRI/utilities/interpolator.h>

using namespace OpenPHRI;

doubleConstPtr Interpolator::getOutput() const {
	return output_;
}

void Interpolator::setInput(doubleConstPtr input) {
	input_ = input;
}
