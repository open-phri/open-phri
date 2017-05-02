#include <RSCL/utilities/fifth_order_polynomial.h>

#include "polynomial_root_finder.h"
#include <iostream>
#include <cmath>

using namespace RSCL;

/* Simplified coefficient for xi = 0 and dx = xf-xi
 * a = -(12*yi - 12*yf + 6*dx*dyf + 6*dx*dyi - d2yf*dx^2 + d2yi*dx^2)/(2*dx^5)
 * b = (30*yi - 30*yf + 14*dx*dyf + 16*dx*dyi - 2*d2yf*dx^2 + 3*d2yi*dx^2)/(2*dx^4)
 * c = -(20*yi - 20*yf + 8*dx*dyf + 12*dx*dyi - d2yf*dx^2 + 3*d2yi*dx^2)/(2*dx^3)
 * d = d2yi/2
 * e = dyi
 * f = yi
 */
void FifthOrderPolynomial::computeParameters(FifthOrderPolynomial::Parameters& params) {

	double dx = params.xf - params.xi;
	double dx_2 = dx*dx, dx_3 = dx_2*dx, dx_4 = dx_3*dx, dx_5 = dx_4*dx;

	params.a =  -(12.*params.yi - 12.*params.yf + 6.*dx*params.dyf + 6.*dx*params.dyi - params.d2yf*dx_2 + params.d2yi*dx_2)/(2.*dx_5);
	params.b =  (30.*params.yi - 30.*params.yf + 14.*dx*params.dyf + 16.*dx*params.dyi - 2.*params.d2yf*dx_2 + 3.*params.d2yi*dx_2)/(2.*dx_4);
	params.c =  -(20.*params.yi - 20.*params.yf + 8.*dx*params.dyf + 12.*dx*params.dyi - params.d2yf*dx_2 + 3.*params.d2yi*dx_2)/(2.*dx_3);
	params.d =  params.d2yi/2.;
	params.e =  params.dyi;
	params.f =  params.yi;
}

double FifthOrderPolynomial::compute(double x, const Parameters& params) {
	double y;

	if(x < params.xi) {
		y = params.yi;
	}
	else if (x > params.xf) {
		y = params.yf;
	}
	else {
		x -= params.xi;
		double x_2 = x*x, x_3 = x_2*x, x_4 = x_3*x, x_5 = x_4*x;
		y = params.a * x_5 + params.b * x_4 + params.c * x_3 + params.d * x_2 + params.e * x + params.f;
	}

	return y;
}

double FifthOrderPolynomial::computeFirstDerivative(double x, const Parameters& params) {
	double dy;

	if(x < params.xi) {
		dy = params.dyi;
	}
	else if (x > params.xf) {
		dy = params.dyf;
	}
	else {
		x -= params.xi;
		double x_2 = x*x, x_3 = x_2*x, x_4 = x_3*x;
		dy = 5.*params.a * x_4 + 4.*params.b * x_3 + 3.*params.c * x_2 + 2.*params.d * x + params.e;
	}

	return dy;
}

double FifthOrderPolynomial::computeSecondDerivative(double x, const Parameters& params) {
	double d2y;

	if(x < params.xi) {
		d2y = params.d2yi;
	}
	else if (x > params.xf) {
		d2y = params.d2yf;
	}
	else {
		x -= params.xi;
		double x_2 = x*x, x_3 = x_2*x;
		d2y = 20.*params.a * x_3 + 12.*params.b * x_2 + 6.*params.c * x + 2.*params.d;
	}

	return d2y;
}

double FifthOrderPolynomial::getFirstDerivativeMaximum(const Parameters& params) {
	int degree = 3;
	double coeffs[MDP1] = {20.*params.a, 12.*params.b, 6.*params.c, 2.*params.d};
	double roots_real[MAXDEGREE];
	double roots_img[MAXDEGREE];

	rpoly_ak1(coeffs, &degree, roots_real, roots_img);

	auto poly = [&params](double x) -> double
				{
					double x_2 = x*x, x_3 = x_2*x, x_4 = x_3*x;
					return std::abs(5.*params.a*x_4 + 4.*params.b*x_3 + 3.*params.c*x_2 + 2.*params.d*x + params.e);
				};

	double max = 0.;
	for (size_t i = 0; i < 3; ++i) {
		max = std::max(max, ( poly(roots_real[i]) ));
	}

	// std::cout << "***    First derivative    ***\n";
	// std::cout << "\tdegree: " << degree << "\n";
	// std::cout << "\troots_real: ";
	// for (size_t i = 0; i < 3; ++i)
	//  std::cout << roots_real[i] << " ";
	// std::cout << "\n\troots_img: ";
	// for (size_t i = 0; i < 3; ++i)
	//  std::cout << roots_img[i] << " ";
	// std::cout << "\n\tmax: " << max;
	// std::cout << std::endl;

	return max;
}

double FifthOrderPolynomial::getSecondDerivativeMaximum(const Parameters& params) {
	int degree = 2;
	double coeffs[MDP1] = {60.*params.a, 24.*params.b, 6.*params.c};
	double roots_real[MAXDEGREE];
	double roots_img[MAXDEGREE];

	rpoly_ak1(coeffs, &degree, roots_real, roots_img);

	auto poly = [&params](double x) -> double
				{
					double x_2 = x*x, x_3 = x_2*x;
					return std::abs(20.*params.a*x_3 + 12.*params.b*x_2 + 6.*params.c*x + 2.*params.d);
				};


	double max = std::max(poly(roots_real[0]), poly(roots_real[1]));

	// std::cout << "***    Second derivative    ***\n";
	// std::cout << "\tdegree: " << degree << "\n";
	// std::cout << "\troots_real: ";
	// for (size_t i = 0; i < 2; ++i)
	//  std::cout << roots_real[i] << " ";
	// std::cout << "\n\troots_img: ";
	// for (size_t i = 0; i < 2; ++i)
	//  std::cout << roots_img[i] << " ";
	// std::cout << "\n\tmax: " << max;
	// std::cout << std::endl;

	return max;
}
