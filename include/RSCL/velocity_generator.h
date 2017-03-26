#pragma once

#include <memory>

#include <RSCL/definitions.h>

namespace RSCL {

class VelocityGenerator {
public:
	VelocityGenerator() = default;
	~VelocityGenerator() = default;

	virtual Vector6d compute() = 0;
};

using VelocityGeneratorPtr = std::shared_ptr<VelocityGenerator>;
using VelocityGeneratorConstPtr = std::shared_ptr<const VelocityGenerator>;

} // namespace RSCL
