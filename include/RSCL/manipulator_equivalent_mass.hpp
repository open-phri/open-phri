#pragma once

#include <RSCL/definitions.h>
#include <RSCL/object_collection.hpp>

namespace RSCL {

template<int joint_count>
class ManipulatorEquivalentMass : public ObjectCollection<Vector6dConstPtr> {
public:
	using InertiaMatrixType = Eigen::Matrix<double, joint_count, joint_count>;
	using JacobianMatrixType = Eigen::Matrix<double, 6, joint_count>;

	ManipulatorEquivalentMass(
		std::shared_ptr<const InertiaMatrixType> inertia_matrix,
		std::shared_ptr<const JacobianMatrixType> jacobian_matrix)
	{
		robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
	}

	ManipulatorEquivalentMass(
		std::shared_ptr<const InertiaMatrixType> inertia_matrix,
		std::shared_ptr<const JacobianMatrixType> jacobian_matrix,
		Vector6dConstPtr robot_position)
	{
		robot_position_ = robot_position;
	}

	~ManipulatorEquivalentMass() = default;

	doubleConstPtr getEquivalentMass() const {
		return mass_;
	}

	double compute() {
		const JacobianMatrixType& jac = *jacobian_matrix_;
		const InertiaMatrixType& inertia = *inertia_matrix_;
		Vector3d direction = closestObjectDirection();

		Matrix6d mass_inv = jac * inertia.inverse() * jac.transpose();

		*mass_ = 1. / (direction.transpose() * mass_inv.block<3,3>(0,0) * direction);

		return *mass_;
	}

private:
	Vector3d closestObjectDirection() {
		Vector3d direction = Vector3d::Zero();
		const Vector3d& rob_pos = robot_position_->block<3,1>(0,0);

		double min_dist = std::numeric_limits<double>::infinity();
		for(const auto& object : objects_) {
			Vector3d obj_rob_vec = object.second->block<3,1>(0,0) - rob_pos;

			double dist =  obj_rob_vec.norm();
			min_dist = std::min(min_dist, dist);
			if(min_dist == dist) {
				direction = obj_rob_vec.normalized();
			}
		}

		return direction;
	}

	std::shared_ptr<const InertiaMatrixType> inertia_matrix_;
	std::shared_ptr<const JacobianMatrixType> jacobian_matrix_;
	Vector6dConstPtr robot_position_;
	doublePtr mass_;
};

} // namespace RSCL
