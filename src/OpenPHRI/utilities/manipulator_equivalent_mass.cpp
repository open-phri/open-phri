#include <OpenPHRI/utilities/manipulator_equivalent_mass.h>

using namespace OpenPHRI;

ManipulatorEquivalentMass::ManipulatorEquivalentMass(
	MatrixXdConstPtr inertia_matrix,
	MatrixXdConstPtr jacobian_matrix) :
	ManipulatorEquivalentMass(inertia_matrix, jacobian_matrix, std::make_shared<Vector6d>(Vector6d::Zero()))
{
}

ManipulatorEquivalentMass::ManipulatorEquivalentMass(
	MatrixXdConstPtr inertia_matrix,
	MatrixXdConstPtr jacobian_matrix,
	Vector6dConstPtr robot_position) :
	inertia_matrix_(inertia_matrix),
	jacobian_matrix_(jacobian_matrix),
	robot_position_(robot_position)
{
	mass_ = std::make_shared<double>(0.);
}

doubleConstPtr ManipulatorEquivalentMass::getEquivalentMass() const {
	return mass_;
}

double ManipulatorEquivalentMass::compute() {
	MatrixXd jac = *jacobian_matrix_;
	MatrixXd inertia = *inertia_matrix_;
	Vector3d direction = closestObjectDirection();

	Matrix6d mass_inv = jac * inertia.inverse() * jac.transpose();

	*mass_ = 1. / (direction.transpose() * mass_inv.block<3,3>(0,0) * direction);

	return *mass_;
}

Vector3d ManipulatorEquivalentMass::closestObjectDirection() {
	Vector3d direction = Vector3d::Zero();
	const Vector3d& rob_pos = robot_position_->block<3,1>(0,0);

	double min_dist = std::numeric_limits<double>::infinity();
	for(const auto& item : items_) {
		Vector6d obj_pos = *item.second;
		Vector3d obj_rob_vec = obj_pos.block<3,1>(0,0) - rob_pos;

		double dist =  obj_rob_vec.norm();
		min_dist = std::min(min_dist, dist);
		if(min_dist == dist) {
			direction = obj_rob_vec.normalized();
		}
	}

	return direction;
}

double ManipulatorEquivalentMass::operator()() {
	return compute();
}
