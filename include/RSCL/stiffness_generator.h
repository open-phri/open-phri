#pragma once

#include <RSCL/force_generator.h>
#include <RSCL/definitions.h>

namespace RSCL {

class StiffnessGenerator : public ForceGenerator {
public:
	StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position);
	StiffnessGenerator(Matrix6dConstPtr stiffness, Vector6dConstPtr target_position, Vector6dConstPtr robot_position);
	~StiffnessGenerator() = default;

	virtual Vector6d compute() override;

private:
	Matrix6dConstPtr stiffness_;
	Vector6dConstPtr target_position_;
	Vector6dConstPtr robot_position_;
};

} // namespace RSCL
