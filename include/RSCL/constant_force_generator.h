#pragma once

#include <force_generator.h>
#include <definitions.h>

namespace RSCL {

class ConstantForceGenerator : public ForceGenerator {
public:
	ConstantForceGenerator(Vector6dConstPtr force);
	~ConstantForceGenerator() = default;

	virtual Vector6d compute() override;

private:
	Vector6dConstPtr force_;
};

} // namespace RSCL
