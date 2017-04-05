#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

class Interpolator {
public:
	Interpolator() = default;
	~Interpolator() = default;

	virtual doubleConstPtr getOutput() const final;
	virtual void setInput(doubleConstPtr input) final;

	virtual void computeParameters() = 0;
	virtual double compute() = 0;

protected:
	doubleConstPtr input_;
	doublePtr output_;
};

using InterpolatorPtr = std::shared_ptr<Interpolator>;
using InterpolatorConstPtr = std::shared_ptr<const Interpolator>;

} // namespace RSCL
