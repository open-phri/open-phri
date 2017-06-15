#include <RSCL/force_generators/potential_field_generator.h>

using namespace RSCL;
using namespace Eigen;

PotentialFieldGenerator::PotentialFieldGenerator()
{
	robot_position_ = std::make_shared<Vector6d>(Vector6d::Zero());
}

PotentialFieldGenerator::PotentialFieldGenerator(
	Vector6dConstPtr robot_position,
	Matrix6dConstPtr spatial_transformation,
	bool do_transpose) :
	robot_position_(robot_position)
{
	spatial_transformation_ = spatial_transformation;
	do_transpose_ = do_transpose;
}

Vector6d PotentialFieldGenerator::compute() {
	Vector3d total_force = Vector3d::Zero();
	const Vector3d& rob_pos = robot_position_->block<3,1>(0,0);

	for(const auto& item : items_) {
		const PotentialFieldObject& obj = *(item.second);
		Vector3d obj_rob_vec = obj.object_position->block<3,1>(0,0) - rob_pos;

		double distance = obj_rob_vec.norm();
		if(std::abs(distance) > 1e-3) {
			Vector3d obj_rob_vec_unit = obj_rob_vec/distance;

			double gain = *obj.gain;

			if(obj.type == PotentialFieldType::Attractive) {
				total_force += gain * obj_rob_vec_unit;
			}
			else {
				double th = *obj.threshold_distance;
				if(distance < th) {
					total_force += gain * (1./th - 1./distance)*obj_rob_vec_unit;
				}
			}
		}
	}


	Vector6d result = Vector6d::Zero();
	result.block<3,1>(0,0) = total_force;

	if(static_cast<bool>(spatial_transformation_)) {
		if(do_transpose_) {
			result = spatial_transformation_->transpose() * result;
		}
		else {
			result = *spatial_transformation_ * result;
		}
	}

	return result;
}
