#pragma once

#include <RSCL/force_generator.h>
#include <RSCL/definitions.h>

namespace RSCL {

class MassGenerator : public ForceGenerator {
public:
	MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration);
	MassGenerator(Matrix6dConstPtr mass, Vector6dConstPtr target_acceleration, Vector6dConstPtr robot_acceleration);
	~MassGenerator() = default;

	virtual Vector6d compute() override;

private:
	Matrix6dConstPtr mass_;
	Vector6dConstPtr target_acceleration_;
	Vector6dConstPtr robot_acceleration_;
};

} // namespace RSCL
