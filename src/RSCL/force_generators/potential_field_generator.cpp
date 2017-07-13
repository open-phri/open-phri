#include <RSCL/force_generators/potential_field_generator.h>

using namespace RSCL;
using namespace Eigen;


PotentialFieldGenerator::PotentialFieldGenerator(
	ReferenceFrame objects_frame) :
	ForceGenerator(objects_frame),
	objects_frame_(objects_frame)
{
	offset_ = std::make_shared<Vector3d>(Vector3d::Zero());
}

PotentialFieldGenerator::PotentialFieldGenerator(
	Vector3dConstPtr offset,
	ReferenceFrame objects_frame) :
	PotentialFieldGenerator(objects_frame)
{
	offset_ = offset;
}

Vector6d PotentialFieldGenerator::compute() {
	Vector3d total_force = Vector3d::Zero();
	Vector3d rob_pos;

	if(objects_frame_ == ReferenceFrame::TCP) {
		rob_pos = *offset_;
	}
	else {
		rob_pos = robot_->controlPointCurrentPose()->block<3,1>(0,0) + robot_->transformationMatrix()->block<3,3>(0,0) * *offset_;
	}

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


	force_ = Vector6d::Zero();
	force_.block<3,1>(0,0) = total_force;

	return ForceGenerator::compute();
}
