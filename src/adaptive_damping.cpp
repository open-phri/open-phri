#include <adaptive_damping.h>
#include <iostream>

using namespace ADamp;
using namespace std;

/***		Constructor & destructor		***/
AdaptiveDamping::AdaptiveDamping (
	std::shared_ptr<Matrix6d> tool_base_transform,
	std::shared_ptr<Matrix6d> damping_matrix,
	std::shared_ptr<Vector6d> reference_velocity,
	std::shared_ptr<Vector6d> force) :
	tool_base_transform_(tool_base_transform),
	damping_matrix_(damping_matrix),
	reference_velocity_(reference_velocity),
	force_(force)
{

}

/***		Algorithm		***/
Vector6d AdaptiveDamping::computeVelocity() {
	computeForces();
	auto alpha = computeConstraint();
	auto h_eq = *force_; // force_ can be updated by the constraints
	auto V_b_t = getToolBaseTransform();
	auto B = getDampingMatrix();
	auto dx_r = getReferenceVelocity();
	Vector6d velocity;

	velocity = V_b_t * (alpha * B.inverse() * h_eq + dx_r);

	return velocity;
}

/***		Helper funtions		***/
void AdaptiveDamping::printState(std::ostream & out) const {
	out << "Constraint value: " << computeConstraint() << std::endl;
	out << "Forces: " << computeForces().transpose() << std::endl;
	out << "Tool-Base transform:\n" << getToolBaseTransform() << std::endl;
	out << "Damping matrix:\n" << getDampingMatrix() << std::endl;
	out << "Reference velocity: " << getReferenceVelocity().transpose() << std::endl;
}

ostream& operator<<(ostream& os, const AdaptiveDamping& ad)
{
	ad.printState(os);
	return os;
}

/***        Setters & getters       ***/
Matrix6d AdaptiveDamping::getToolBaseTransform() const {
	return *tool_base_transform_;

}

void AdaptiveDamping::setToolBaseTransform(const Matrix6d& transform) {
	*tool_base_transform_ = transform;
}

Matrix6d AdaptiveDamping::getDampingMatrix() const {
	return *damping_matrix_;
}

void AdaptiveDamping::setDampingMatrix(const Matrix6d& damping) {
	*damping_matrix_ = damping;
}

Vector6d AdaptiveDamping::getReferenceVelocity() const {
	return *reference_velocity_;
}

void AdaptiveDamping::setReferenceVelocity(const Vector6d& velocity) {
	*reference_velocity_ = velocity;
}

double AdaptiveDamping::computeConstraint() const {
	return 0.5;
}

Vector6d AdaptiveDamping::computeForces() const {
	return Vector6d::Ones();
}
