#pragma once

#include <RSCL/force_generator.h>
#include <RSCL/definitions.h>

namespace RSCL {

class ForceProxy : public ForceGenerator {
public:
	ForceProxy(Vector6dConstPtr force);
	~ForceProxy() = default;

	virtual Vector6d compute() override;

private:
	Vector6dConstPtr force_;
};

} // namespace RSCL
