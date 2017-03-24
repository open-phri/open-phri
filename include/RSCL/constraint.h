#pragma once

#include <memory>

namespace RSCL {

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

using ConstraintPtr = std::shared_ptr<Constraint>;
using ConstraintConstPtr = std::shared_ptr<const Constraint>;

} // namespace RSCL
