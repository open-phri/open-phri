#pragma once

#include <velocity_generator.h>
#include <definitions.h>

namespace RSCL {

class ConstantVelocityGenerator : public VelocityGenerator {
public:
	ConstantVelocityGenerator(Vector6dConstPtr velocity);
	~ConstantVelocityGenerator() = default;

	virtual Vector6d compute() override;

private:
	Vector6dConstPtr velocity_;
};

} // namespace RSCL
