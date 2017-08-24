#include <OpenPHRI/utilities/polynomial_interpolator.h>

using namespace phri;

PolynomialInterpolator::PolynomialInterpolator(
	PolynomialPointConstPtr from,
	PolynomialPointConstPtr to,
	doubleConstPtr input) :
	PolynomialInterpolator(from, to)
{
	setInput(input);
}

PolynomialInterpolator::PolynomialInterpolator(
	PolynomialPointConstPtr from,
	PolynomialPointConstPtr to)
{
	from_ = from;
	to_ = to;
	output_ = std::make_shared<double>(0.);

	computeParameters();
}

void PolynomialInterpolator::computeParameters() {
	double xi = *from_->x, xf = *to_->x;
	double yi = *from_->y, yf = *to_->y;
	double dyi = *from_->dy, dyf = *to_->dy;
	double d2yi = *from_->d2y, d2yf = *to_->d2y;
	params_ = {xi, xf, yi, yf, dyi, dyf, d2yi, d2yf};
	FifthOrderPolynomial::computeParameters(params_);
}

double PolynomialInterpolator::compute() {
	*output_ = FifthOrderPolynomial::compute(*input_, params_);

	return *output_;
}
