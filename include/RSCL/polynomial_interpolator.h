#pragma once

#include <RSCL/interpolator.h>

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


class PolynomialInterpolator : public Interpolator {
public:
	PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to, doubleConstPtr input);
	PolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to);
	~PolynomialInterpolator() = default;

	virtual void computeParameters() override;
	virtual double compute() override;

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
};

using PolynomialInterpolatorPtr = std::shared_ptr<PolynomialInterpolator>;
using PolynomialInterpolatorConstPtr = std::shared_ptr<const PolynomialInterpolator>;

} // namespace RSCL
