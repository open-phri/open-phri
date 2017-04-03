#include <RSCL/potential_field_generator.h>

#include <iostream>

using namespace RSCL;
using namespace Eigen;

PotentialFieldGenerator::PotentialFieldGenerator(Vector6dConstPtr robot_position) : robot_position_(robot_position) {

}

void PotentialFieldGenerator::setVerbose(bool on) {
	verbose_ = on;
}

Vector6d PotentialFieldGenerator::compute() {
	Vector3d total_force = Vector3d::Zero();
	Vector3d rob_pos;
	if(robot_position_) {
		rob_pos = robot_position_->block<3,1>(0,0);
	}
	else {
		rob_pos = Vector3d::Zero();
	}

	for(const auto& object : objects_) {
		const PotentialFieldObject& obj = *(object.second);
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
	return result;
}

bool PotentialFieldGenerator::addObject(const std::string& name, PotentialFieldObjectPtr object, bool force) {
	if((objects_.find(name) != objects_.end())and not force) {
		if(verbose_) {
			std::cerr << "In PotentialFieldGenerator::addObject: an object called \"" << name << "\" already exists. Not replaced (force = false)" << std::endl;
		}
		return false;
	}
	objects_[name] = object;
	return true;
}

bool PotentialFieldGenerator::removeObject(const std::string& name) {
	auto elem = objects_.find(name);
	if(elem == objects_.end()) {
		if(verbose_) {
			std::cerr << "In PotentialFieldGenerator::removeObject: no object called \"" << name << "\"" << std::endl;
		}
		return false;
	}
	objects_.erase(elem);
	return true;
}

PotentialFieldObjectPtr PotentialFieldGenerator::getObject(const std::string& name) {
	PotentialFieldObjectPtr ptr;
	auto elem = objects_.find(name);
	if(elem != objects_.end()) {
		ptr = elem->second;
	}
	else if(verbose_) {
		std::cerr << "In PotentialFieldGenerator::getObject: no object called \"" << name << "\"" << std::endl;
	}
	return ptr;
}
