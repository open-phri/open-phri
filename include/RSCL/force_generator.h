#pragma once

#include <definitions.h>

namespace RSCL {

class ForceGenerator {
public:
	ForceGenerator() = default;
	~ForceGenerator() = default;

	virtual Vector6d compute() = 0;
};

using ForceGeneratorPtr = std::shared_ptr<ForceGenerator>;
using ForceGeneratorConstPtr = std::shared_ptr<const ForceGenerator>;

} // namespace RSCL
