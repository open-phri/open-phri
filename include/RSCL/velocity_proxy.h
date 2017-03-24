#pragma once

#include <velocity_generator.h>
#include <definitions.h>

namespace RSCL {

class VelocityProxy : public VelocityGenerator {
public:
	VelocityProxy(Vector6dConstPtr velocity);
	~VelocityProxy() = default;

	virtual Vector6d compute() override;

private:
	Vector6dConstPtr velocity_;
};

} // namespace RSCL
