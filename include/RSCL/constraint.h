#pragma once

#include <memory>

namespace RSCL {

namespace Constraints {

enum class ConstraintType {
	Multiplicative,
	Minimum
};

class Constraint {
public:
	Constraint(ConstraintType type);
	virtual ~Constraint() = default;

	ConstraintType getType();
	virtual double compute() = 0;

protected:
	ConstraintType type_;
};

} // namespace RSCL

using ConstraintPtr = std::shared_ptr<Constraints::Constraint>;

} // namespace Constraints
