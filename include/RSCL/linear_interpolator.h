#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

class LinearInterpolator {
public:
	LinearInterpolator(Vector2dConstPtr from, Vector2dConstPtr to, doubleConstPtr input);
	~LinearInterpolator() = default;

	doubleConstPtr getOutput() const;

	void enableSaturation(bool on);
	void computeParameters();
	double compute();

private:
	struct PolynomialParameters {
		double a;
		double b;

		double xi;
		double xf;
		double yi;
		double yf;
	};

	PolynomialParameters params_;

	Vector2dConstPtr from_;
	Vector2dConstPtr to_;
	doubleConstPtr input_;
	doublePtr output_;
	bool saturation_;
};

} // namespace RSCL
