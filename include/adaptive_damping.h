#ifndef ADAPTIVE_DAMPING_H
#define ADAPTIVE_DAMPING_H

#include <memory>
#include <iostream>

#include <definitions.h>

namespace ADamp {

class AdaptiveDamping {
public:
	/***		Constructor & destructor		***/
	AdaptiveDamping(
		std::shared_ptr<Matrix6d> tool_base_transform,
		std::shared_ptr<Matrix6d> damping_matrix,
		std::shared_ptr<Vector6d> reference_velocity,
		std::shared_ptr<Vector6d> force);

	virtual ~AdaptiveDamping() = default;

	/***		Algorithm		***/
	Vector6d computeVelocity();

	/***		Helper funtions		***/
	void printState(std::ostream & out = std::cout) const;
	friend std::ostream& operator<<(std::ostream& os, const AdaptiveDamping& ad);

	/***        Setters & getters       ***/
	Matrix6d getToolBaseTransform() const;
	void setToolBaseTransform(const Matrix6d& transform);

	Matrix6d getDampingMatrix() const;
	void setDampingMatrix(const Matrix6d& damping);

	Vector6d getReferenceVelocity() const;
	void setReferenceVelocity(const Vector6d& velocity);

protected:
	virtual double computeConstraint() const;

	virtual Vector6d computeForces() const;

private:
	std::shared_ptr<Matrix6d> tool_base_transform_;
	std::shared_ptr<Matrix6d> damping_matrix_;
	std::shared_ptr<Vector6d> reference_velocity_;
	std::shared_ptr<Vector6d> force_;

};

}

#endif /* ADAPTIVE_DAMPING_H */
