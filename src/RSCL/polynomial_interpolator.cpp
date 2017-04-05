#include <RSCL/polynomial_interpolator.h>

using namespace RSCL;

PolynomialInterpolator::PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to, doubleConstPtr input) {
	from_ = from;
	to_ = to;
	input_ = input;
	output_ = std::make_shared<double>(0.);

	computeParameters();
}

doubleConstPtr PolynomialInterpolator::getOutput() const {
	return output_;
}

/* Simplified coefficient for xi = 0 and dx = xf-xi
 * a = -(12*yi - 12*yf + 6*dx*dyf + 6*dx*dyi - d2yf*dx^2 + d2yi*dx^2)/(2*dx^5)
 * b = (30*yi - 30*yf + 14*dx*dyf + 16*dx*dyi - 2*d2yf*dx^2 + 3*d2yi*dx^2)/(2*dx^4)
 * c = -(20*yi - 20*yf + 8*dx*dyf + 12*dx*dyi - d2yf*dx^2 + 3*d2yi*dx^2)/(2*dx^3)
 * d = d2yi/2
 * e = dyi
 * f = yi
 */
void PolynomialInterpolator::computeParameters() {
	double xi = *from_->x, xf = *to_->x;
	double yi = *from_->y, yf = *to_->y;
	double dyi = *from_->dy, dyf = *to_->dy;
	double d2yi = *from_->d2y, d2yf = *to_->d2y;
	double dx = xf - xi;
	double dx_2 = dx*dx, dx_3 = dx_2*dx, dx_4 = dx_3*dx, dx_5 = dx_4*dx;

	params_.a =  -(12.*yi - 12.*yf + 6.*dx*dyf + 6.*dx*dyi - d2yf*dx_2 + d2yi*dx_2)/(2.*dx_5);
	params_.b =  (30.*yi - 30.*yf + 14.*dx*dyf + 16.*dx*dyi - 2.*d2yf*dx_2 + 3.*d2yi*dx_2)/(2.*dx_4);
	params_.c =  -(20.*yi - 20.*yf + 8.*dx*dyf + 12.*dx*dyi - d2yf*dx_2 + 3.*d2yi*dx_2)/(2.*dx_3);
	params_.d =  d2yi/2.;
	params_.e =  dyi;
	params_.f =  yi;

	params_.xi = xi;
	params_.xf = xf;
	params_.yi = yi;
	params_.yf = yf;
}

double PolynomialInterpolator::compute() {
	double x = *input_;
	double y;

	if(x < params_.xi) {
		y = params_.yi;
	}
	else if (x > params_.xf) {
		y = params_.yf;
	}
	else {
		x -= params_.xi;
		double x_2 = x*x, x_3 = x_2*x, x_4 = x_3*x, x_5 = x_4*x;
		y = params_.a * x_5 + params_.b * x_4 + params_.c * x_3 + params_.d * x_2 + params_.e * x + params_.f;
	}

	*output_ = y;
	return y;
}
