#include <OpenPHRI/utilities/linear_interpolator.h>

using namespace phri;

LinearInterpolator::LinearInterpolator(
	LinearPointConstPtr from,
	LinearPointConstPtr to,
	doubleConstPtr input) :
	LinearInterpolator(from, to)
{
	setInput(input);
}

LinearInterpolator::LinearInterpolator(
	LinearPointConstPtr from,
	LinearPointConstPtr to)
{
	from_ = from;
	to_ = to;
	output_ = std::make_shared<double>(0.);
	saturation_ = false;

	computeParameters();
}

void LinearInterpolator::enableSaturation(bool on) {
	saturation_ = on;
}

void LinearInterpolator::computeParameters() {
	params_.xi = *from_->x;
	params_.xf = *to_->x;
	params_.yi = *from_->y;
	params_.yf = *to_->y;

	params_.a = (params_.yf - params_.yi)/(params_.xf - params_.xi);
	params_.b = params_.yi - params_.a*params_.xi;
}

double LinearInterpolator::compute() {
	double x = *input_;
	double y;

	if(saturation_) {
		if(x < params_.xi) {
			y = params_.yi;
		}
		else if (x > params_.xf) {
			y = params_.yf;
		}
		else {
			y = params_.a*x + params_.b;
		}
	}
	else {
		y = params_.a*x + params_.b;
	}

	*output_ = y;
	return y;
}
