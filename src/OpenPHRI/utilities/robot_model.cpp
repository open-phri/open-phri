#include <OpenPHRI/utilities/robot_model.h>
#include <OpenPHRI/utilities/exceptions.h>

// RBDyn
#include <RBDyn/Body.h>
#include <RBDyn/FK.h>
#include <RBDyn/FV.h>
#include <RBDyn/Joint.h>
#include <RBDyn/Jacobian.h>
#include <RBDyn/MultiBody.h>
#include <RBDyn/MultiBodyGraph.h>
#include <RBDyn/MultiBodyConfig.h>
#include <rbdyn-parsers/yaml.h>
#include <rbdyn-parsers/urdf.h>

#include <yaml-cpp/yaml.h>
#include <pid/rpath.h>

#include <iostream>

using namespace phri;

struct RobotModel::pImpl {
	pImpl(RobotPtr robot,
	      const std::string& model_path,
	      const std::string& control_point) :
		robot(robot)
	{
		auto file = PID_PATH(model_path);
		auto extension_pos = file.rfind('.');
		auto extension = file.substr(extension_pos+1);
		rbd::ParserResult parser_result;
		if(extension == "yaml" or extension == "yml") {
			parser_result = rbd::RBDynFromYAML(file, rbd::ParserInput::File, true).result(); // fixed base robot
		}
		else if(extension == "urdf") {
			parser_result = rbd::RBDynFromURDF(file, rbd::ParserInput::File, true).result(); // fixed base robot
		}
		else {
			throw std::runtime_error(OPEN_PHRI_ERROR("Unkown robot model extension '" + extension + "'. Please provide a yaml, yml or urdf file."));
		}
		mb = parser_result.mb;
		mbc = parser_result.mbc;
		mbg = parser_result.mbg;
		name = parser_result.name;
		auto indexes = mb.jointIndexByName();
		size_t joint_count = mbc.q.size() - 1;

		lower_limit = std::make_shared<VectorXd>(joint_count);
		upper_limit = std::make_shared<VectorXd>(joint_count);
		velocity_limit = std::make_shared<VectorXd>(joint_count);
		force_limit = std::make_shared<VectorXd>(joint_count);

		for(const auto& index: indexes) {
			if(index.second == 0) {
				// skip root joint
				continue;
			}
			(*lower_limit)(index.second - 1) = parser_result.limits.lower[index.first][0];
			(*upper_limit)(index.second - 1) = parser_result.limits.upper[index.first][0];
			(*velocity_limit)(index.second - 1) = parser_result.limits.velocity[index.first][0];
			(*force_limit)(index.second - 1) = parser_result.limits.torque[index.first][0];
		}
		jacobian = rbd::Jacobian(mb, control_point);
		control_point_body_index = mb.bodyIndexByName(control_point);
	}

	void updateJointPositions() {
		const auto& joint_pos = *robot->jointCurrentPosition();
		for (size_t i = 0; i < robot->jointCount(); ++i) {
			mbc.q[i+1][0] = joint_pos(i);
		}
	}

	void updateSpatialTransformation() {
		auto& mat = *robot->spatialTransformationMatrix();
		const auto& rot_mat = robot->transformationMatrix()->block<3,3>(0,0);
		mat.block<3,3>(3,0).setZero();
		mat.block<3,3>(0,0) = rot_mat;
		mat.block<3,3>(0,3).setZero();
		mat.block<3,3>(3,3) = rot_mat;
	}

	void forwardKinematics() {
		updateJointPositions();
		rbd::forwardKinematics(mb, mbc);
		rbd::forwardVelocity(mb, mbc);

		const auto& tcp_pose = mbc.bodyPosW[control_point_body_index];
		*robot->controlPointCurrentPose() = Pose(tcp_pose.translation(), static_cast<Eigen::Quaterniond>(tcp_pose.rotation().transpose()));
		robot->transformationMatrix()->setIdentity();
		robot->transformationMatrix()->block<3,1>(0,3) = tcp_pose.translation();
		robot->transformationMatrix()->block<3,3>(0,0) = tcp_pose.rotation().transpose();
		updateSpatialTransformation();

		auto joint_count = robot->jointCount();
		Eigen::MatrixXd jac_mat = jacobian.jacobian(mb, mbc);
		robot->jacobian()->block(0, 0, 3, joint_count) = jac_mat.block(3, 0, 3, joint_count);
		robot->jacobian()->block(3, 0, 3, joint_count) = jac_mat.block(0, 0, 3, joint_count);
	}


	rbd::MultiBody mb;
	rbd::MultiBodyConfig mbc;
	rbd::MultiBodyGraph mbg;
	rbd::Jacobian jacobian;
	std::string name;

	RobotPtr robot;
	int control_point_body_index;
	VectorXdPtr lower_limit;
	VectorXdPtr upper_limit;
	VectorXdPtr velocity_limit;
	VectorXdPtr force_limit;
};

RobotModel::RobotModel(
	RobotPtr robot,
	const std::string& model_path,
	const std::string& control_point) :
	impl_(std::make_unique<RobotModel::pImpl>(robot, model_path, control_point))
{

}

RobotModel::RobotModel(RobotPtr robot, const YAML::Node& configuration) {
	const auto& model = configuration["model"];
	std::string path;
	std::string control_point;
	if(model) {
		try {
			path = model["path"].as<std::string>();
		}
		catch(...) {
			throw std::runtime_error(OPEN_PHRI_ERROR("You must provide a 'path' field in the model configuration."));
		}
		try {
			control_point = model["control_point"].as<std::string>();
		}
		catch(...) {
			throw std::runtime_error(OPEN_PHRI_ERROR("You must provide a 'control_point' field in the model configuration."));
		}

		impl_= std::make_unique<RobotModel::pImpl>(robot, path, control_point);
	}
	else {
		throw std::runtime_error(OPEN_PHRI_ERROR("The configuration file doesn't include a 'model' field."));
	}
}

RobotModel::~RobotModel() = default;

void RobotModel::forwardKinematics() const {
	impl_->forwardKinematics();
}

VectorXdConstPtr RobotModel::getLowerLimits() const {
	return impl_->lower_limit;
}
VectorXdConstPtr RobotModel::getUpperLimits() const {
	return impl_->upper_limit;
}
VectorXdConstPtr RobotModel::getVelocityLimits() const {
	return impl_->velocity_limit;
}
VectorXdConstPtr RobotModel::getForceLimits() const {
	return impl_->force_limit;
}
size_t RobotModel::jointCount() const {
	return impl_->lower_limit->size();
}
const std::string& RobotModel::name() const {
	return impl_->name;
}
