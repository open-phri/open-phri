#pragma once

#include <RSCL/interpolator.h>

namespace RSCL {

class LinearInterpolator : public Interpolator {
public:
	LinearInterpolator(Vector2dConstPtr from, Vector2dConstPtr to, doubleConstPtr input);
	LinearInterpolator(Vector2dConstPtr from, Vector2dConstPtr to);
	~LinearInterpolator() = default;

	void enableSaturation(bool on);
	virtual void computeParameters() override;
	virtual double compute() override;

private:
	struct LinearParameters {
		double a;
		double b;

		double xi;
		double xf;
		double yi;
		double yf;
	};

	LinearParameters params_;

	Vector2dConstPtr from_;
	Vector2dConstPtr to_;
	bool saturation_;
};

using LinearInterpolatorPtr = std::shared_ptr<LinearInterpolator>;
using LinearInterpolatorConstPtr = std::shared_ptr<const LinearInterpolator>;

} // namespace RSCL
