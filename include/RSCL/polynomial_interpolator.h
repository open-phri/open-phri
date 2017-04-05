#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

struct PolynomialPoint {
	PolynomialPoint(
		doubleConstPtr x,
		doubleConstPtr y,
		doubleConstPtr dy,
		doubleConstPtr d2y) :
		x(x), y(y), dy(dy), d2y(d2y)
	{
	}

	doubleConstPtr x;   // x value
	doubleConstPtr y;   // y value
	doubleConstPtr dy;  // first derivative
	doubleConstPtr d2y; // second derivative
};

using PolynomialPointPtr = std::shared_ptr<PolynomialPoint>;
using PolynomialPointConstPtr = std::shared_ptr<const PolynomialPoint>;


class PolynomialInterpolator {
public:
	PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to, doubleConstPtr input);
	~PolynomialInterpolator() = default;

	doubleConstPtr getOutput() const;

	void computeParameters();
	double compute();

private:
	struct PolynomialParameters {
		double a;
		double b;
		double c;
		double d;
		double e;
		double f;

		double xi;
		double xf;
		double yi;
		double yf;
	};

	PolynomialParameters params_;

	PolynomialPointConstPtr from_;
	PolynomialPointConstPtr to_;
	doubleConstPtr input_;
	doublePtr output_;
};

} // namespace RSCL
