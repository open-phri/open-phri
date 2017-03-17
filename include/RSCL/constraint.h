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
	ConstraintType getType();
	virtual double compute() = 0;

protected:
	Constraint(ConstraintType type);
	virtual ~Constraint() = default;

	ConstraintType type_;
};

} // namespace RSCL

using ConstraintPtr = std::shared_ptr<Constraints::Constraint>;

} // namespace Constraints
