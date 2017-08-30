#include <vrep_driver/vrep_driver.h>

#include <stdexcept>
#include <sstream>
#include <iostream>
#include <vector>

#include <extApi.h>
#include <v_repConst.h>

using namespace vrep;
using namespace std;

VREPDriver::VREPDriver(
	phri::RobotPtr robot,
	ControlLevel control_level,
	double sample_time,
	const std::string& suffix,
	const std::string& ip,
	int port) :
	control_level_(control_level),
	sample_time_(sample_time),
	sync_mode_(false),
	robot_(robot),
	suffix_(suffix)
{
	init(ip, port);
}

VREPDriver::VREPDriver(
	phri::RobotPtr robot,
	ControlLevel control_level,
	double sample_time,
	int client_id,
	const std::string& suffix) :
	control_level_(control_level),
	sample_time_(sample_time),
	sync_mode_(false),
	robot_(robot),
	suffix_(suffix)
{
	init(client_id);
}

VREPDriver::~VREPDriver() {
	enableSynchonous(false);
}

void VREPDriver::init(const std::string& ip, int port) {
	client_id_ = simxStart((simxChar*)ip.c_str(), port, 0, 1, 10000, int(sample_time_*1000));

	if(client_id_ != -1) {
		return init(client_id_);
	}
	else {
		simxFinish(client_id_);
		throw std::runtime_error("VREPDriver::init: can't initialize the connection with V-REP");
	}
}

void VREPDriver::init(int client_id) {
	assert_msg("In VREPDriver::init: invalid client id", client_id >= 0);
	client_id_ = client_id;

	getObjectHandles();
	startStreaming();
}


int VREPDriver::getClientID() const {
	return client_id_;
}

bool VREPDriver::checkConnection() const {
	return (simxGetConnectionId(client_id_) != -1);
}

bool VREPDriver::enableSynchonous(bool state) {
	sync_mode_ = state;
	return (simxSynchronous(client_id_, sync_mode_) == simx_return_ok);
}

bool VREPDriver::nextStep() const {
	if(sync_mode_) {
		return (simxSynchronousTrigger(client_id_) == simx_return_ok);
	}
	return false;
}

void VREPDriver::startSimulation() const {
	simxStartSimulation(client_id_, simx_opmode_oneshot_wait);
	if(sync_mode_) {
		nextStep();
		nextStep();
	}
}

void VREPDriver::stopSimulation() const {
	simxStopSimulation(client_id_, simx_opmode_oneshot_wait);
}

void VREPDriver::pauseSimulation() const {
	simxPauseSimulation(client_id_, simx_opmode_oneshot_wait);
}

bool VREPDriver::readTCPPose(phri::Vector6dPtr pose, phri::ReferenceFrame frame) const {
	bool all_ok = true;
	float data[6];

	int object_handle = object_handles_.at(robot_->name() + "_tcp" + suffix_);
	int frame_id = getFrameHandle(frame);
	all_ok &= (simxGetObjectPosition    (client_id_, object_handle, frame_id, data,    simx_opmode_buffer) == simx_return_ok);
	all_ok &= (simxGetObjectOrientation (client_id_, object_handle, frame_id, data+3,  simx_opmode_buffer) == simx_return_ok);

	if(all_ok) {
		double* pose_data = pose->data();
		for (size_t i = 0; all_ok and i < 6; ++i) {
			pose_data[i] = data[i];
		}
	}

	return all_ok;
}

bool VREPDriver::readTCPVelocity(phri::Vector6dPtr velocity, phri::ReferenceFrame frame) const {
	using namespace Eigen;

	bool all_ok = true;
	float data[6], angles[3];

	int object_handle = object_handles_.at(robot_->name() + "_tcp" + suffix_);
	int frame_id = getFrameHandle(frame);
	all_ok &= (simxGetObjectOrientation (client_id_, frame_id, -1, angles,  simx_opmode_buffer) == simx_return_ok);
	all_ok &= (simxGetObjectVelocity(client_id_, object_handle, data, data+3, simx_opmode_buffer) == simx_return_ok);

	if(all_ok) {
		double* velocity_data = velocity->data();
		for (size_t i = 0; all_ok and i < 6; ++i) {
			velocity_data[i] = data[i];
		}

		// With V-REP, the velocity is (sadly) always expressed in the absolute frame so we need to map it from the absolute frame to the given frame
		Matrix3d rot_mat;
		rot_mat =   AngleAxisd(angles[0], Vector3d::UnitX())
		          * AngleAxisd(angles[1], Vector3d::UnitY())
		          * AngleAxisd(angles[2], Vector3d::UnitZ());

		velocity->block<3,1>(0,0) = rot_mat.transpose() * velocity->block<3,1>(0,0);
		velocity->block<3,1>(3,0) = rot_mat.transpose() * velocity->block<3,1>(3,0);
	}

	return all_ok;
}

bool VREPDriver::readTCPTargetPose(phri::Vector6dPtr pose, phri::ReferenceFrame frame) const {
	bool all_ok = true;
	float data[6];

	int object_handle = object_handles_.at(robot_->name() + "_tcp_target" + suffix_);
	int frame_id = getFrameHandle(frame);
	all_ok &= (simxGetObjectPosition    (client_id_, object_handle, frame_id, data,    simx_opmode_buffer) == simx_return_ok);
	all_ok &= (simxGetObjectOrientation (client_id_, object_handle, frame_id, data+3,  simx_opmode_buffer) == simx_return_ok);

	if(all_ok) {
		double* pose_data = pose->data();
		for (size_t i = 0; all_ok and i < 6; ++i) {
			pose_data[i] = data[i];
		}
	}

	return all_ok;
}

bool VREPDriver::sendTCPtargetVelocity(phri::Vector6dConstPtr velocity, phri::ReferenceFrame frame) const {
	bool all_ok = true;

	auto pose = make_shared<phri::Vector6d>();
	all_ok &= readTCPTargetPose(pose, frame);
	*pose += *velocity * sample_time_;

	float data[6];
	double* pose_data = pose->data();
	for (size_t i = 0; all_ok and i < 6; ++i) {
		data[i] = pose_data[i];
	}
	int object_handle = object_handles_.at(robot_->name() + "_tcp_target" + suffix_);
	int frame_id = getFrameHandle(frame);
	all_ok &= (simxSetObjectPosition    (client_id_, object_handle, frame_id, data,    simx_opmode_oneshot) != -1);
	all_ok &= (simxSetObjectOrientation (client_id_, object_handle, frame_id, data+3,  simx_opmode_oneshot) != -1);

	return all_ok;
}

bool VREPDriver::readTCPWrench(phri::Vector6dPtr wrench) const {
	bool all_ok = true;
	float data[6];
	uint8_t ft_state;
	string obj_name = robot_->name() + "_force_sensor" + suffix_;

	all_ok &= (simxReadForceSensor(client_id_, object_handles_.at(obj_name), &ft_state, data, data+3, simx_opmode_buffer) == simx_return_ok);
	all_ok &= ft_state == 0b01; // ft not broken + data available

	if(all_ok) {
		double* wrench_data = wrench->data();
		for (size_t i = 0; all_ok and i < 6; ++i) {
			wrench_data[i] = -data[i]; // V-REP inverts the measured forces
		}
	}

	return all_ok;
}

bool VREPDriver::readJacobian(phri::MatrixXdPtr jacobian) const {
	bool all_ok = false;

	simxUChar* jacobian_buf;
	simxInt sLength;
	int ret = simxReadStringStream(client_id_, ("Jacobian-"+robot_->name()).c_str(), &jacobian_buf, &sLength, simx_opmode_buffer);
	if (ret == simx_return_ok) {
		if(sLength == 0) {
			return false;
		}

		std::string jacobian_str = std::string((char*)(jacobian_buf));
		std::istringstream iss(jacobian_str);
		std::vector<std::string> lines;

		size_t rows, cols;
		iss >> rows;
		iss >> cols;
		jacobian->resize(rows, cols);
		auto& jac = *jacobian;
		for(size_t idx = 0; idx < rows*cols; ++idx) {
			size_t r = idx/cols, c = idx%cols;
			iss >> jac(r,c);
		}
		// Jacobians in V-REP are transposed compared to the standard form and with joints in the tip-to-base order so we fix all that
		jac = jac.transpose().rowwise().reverse().eval();
		all_ok = true;
	}
	// else {
	//  std::cerr << "JACOBIAN ERROR! ret: " << ret << "\n";
	// }

	return all_ok;
}

bool VREPDriver::readTransformationMatrix(phri::Matrix4dPtr matrix) const {
	bool all_ok = false;

	simxUChar* matrix_buf;
	simxInt sLength;
	if (simxReadStringStream(client_id_, ("RotMat-"+robot_->name()).c_str(), &matrix_buf, &sLength, simx_opmode_buffer) == simx_return_ok) {
		std::string matrix_str = std::string((char*)(matrix_buf));
		std::istringstream iss(matrix_str);

		auto& mat = *matrix;
		mat.setIdentity();
		for (size_t row = 0; row < 3; ++row) {
			for (size_t col = 0; col < 4; ++col) {
				iss >> mat(row,col);
			}
		}
		all_ok = true;
	}

	return all_ok;
}

phri::Vector6dConstPtr VREPDriver::trackObjectPosition(const std::string& name, phri::ReferenceFrame frame) {
	int handle = -1;
	int ref_frame = getFrameHandle(frame);
	float data[3];

	if(simxGetObjectHandle(client_id_, name.c_str(), &handle, simx_opmode_oneshot_wait) != simx_return_ok) {
		cerr << "In VREPDriver::trackObjectPosition: can't get the handle of object " << name << endl;
	}

	simxGetObjectPosition(client_id_, handle, ref_frame, data, simx_opmode_streaming);

	auto ptr = make_shared<phri::Vector6d>(phri::Vector6d::Zero());

	tracked_objects_[make_pair(handle, ref_frame)] = ptr;

	return ptr;
}

bool VREPDriver::updateTrackedObjectsPosition() {
	bool all_ok = true;
	for(auto& obj: tracked_objects_) {
		float data[3];
		all_ok &= (simxGetObjectPosition(client_id_, obj.first.first, obj.first.second, data, simx_opmode_buffer) == simx_return_ok);
		if(all_ok) {
			double* pose_data = obj.second->data();
			for (size_t i = 0; all_ok and i < 3; ++i) {
				pose_data[i] = data[i];
			}
		}
		else {
			cerr << "In VREPDriver::updateTrackedObjectsPosition: can't get position of object with handle " << obj.first.first << endl;
		}
	}
	return all_ok;
}

phri::VectorXdConstPtr VREPDriver::initLaserScanner(const std::string& name) {
	std::string data_name = name + "_data";
	simxUChar* sigVal;
	simxInt sigLen;

	simxReadStringStream(client_id_, data_name.c_str(), &sigVal, &sigLen, simx_opmode_streaming);

	auto ptr = std::make_shared<phri::VectorXd>();
	lasers_data_[data_name] = ptr;

	return ptr;
}

bool VREPDriver::updateLaserScanners() {
	bool all_ok = true;

	for (auto& laser: lasers_data_) {
		simxUChar* sigVal;
		simxInt sigLen;

		if(simxReadStringStream(client_id_, laser.first.c_str(), &sigVal, &sigLen, simx_opmode_buffer) == simx_return_ok) {
			size_t count = sigLen / sizeof(float);

			auto& vec = *laser.second;
			if(count != vec.size()) {
				vec.resize(count);
			}

			float* distances = reinterpret_cast<float*>(sigVal);
			for (size_t i = 0; i < count; ++i) {
				vec(i) = distances[i];
			}
		}
		else {
			all_ok = false;
		}
	}

	return all_ok;
}

bool VREPDriver::readJointPosition(phri::VectorXdPtr position) const {
	bool all_ok = true;

	float positions[robot_->jointCount()];

	for (size_t i = 0; i < robot_->jointCount(); ++i) {
		int joint_handle = object_handles_.at(robot_->name() + "_joint" + std::to_string(i+1) + suffix_);
		all_ok &= (simxGetJointPosition(client_id_, joint_handle, positions+i, simx_opmode_buffer) != -1);
	}
	double* position_data = robot_->jointCurrentPosition()->data();
	for (size_t i = 0; all_ok and i < robot_->jointCount(); ++i) {
		position_data[i] = positions[i];
	}

	return all_ok;
}

bool VREPDriver::sendJointTargetPosition(phri::VectorXdConstPtr position) const {
	bool all_ok = true;

	const auto& position_data = *position;
	for (size_t i = 0; i < robot_->jointCount(); ++i) {
		int joint_handle = object_handles_.at(robot_->name() + "_joint" + std::to_string(i+1) + suffix_);
		all_ok &= (simxSetJointPosition(client_id_, joint_handle, position_data(i), simx_opmode_oneshot) != -1);
	}

	return all_ok;
}

bool VREPDriver::sendJointTargetVelocity(phri::VectorXdConstPtr velocity) const {
	bool all_ok = true;

	const auto& velocity_vec = *velocity;
	const auto& position_vec = *robot_->jointCurrentPosition();

	*robot_->jointTargetPosition() += velocity_vec*sample_time_;

	all_ok &= sendJointTargetPosition(robot_->jointTargetPosition());

	return all_ok;
}


bool VREPDriver::getObjectHandles() {
	bool all_ok = true;

	auto getHandle =
		[this](const std::string& name) -> bool {
			string obj_name = robot_->name() + "_" + name + suffix_;
			bool ok = simxGetObjectHandle(client_id_, obj_name.c_str(), &object_handles_[obj_name], simx_opmode_oneshot_wait) == simx_return_ok;
			if(not ok) {
				cerr << "In VREPDriver::getObjectHandles: can't get the handle of object " << obj_name << endl;
			}
			return ok;
		};

	all_ok &= getHandle("tcp");
	all_ok &= getHandle("tcp_target");
	all_ok &= getHandle("base_frame");
	all_ok &= getHandle("world_frame");
	all_ok &= getHandle("force_sensor");

	for (size_t i = 1; i <= robot_->jointCount(); ++i) {
		all_ok &= getHandle("joint" + std::to_string(i));
	}

	return all_ok;
}

void VREPDriver::startStreaming() const {
	float data[6];

	phri::ReferenceFrame frames[] = {phri::ReferenceFrame::TCP, phri::ReferenceFrame::Base, phri::ReferenceFrame::World};
	string objects[] = {"_tcp", "_tcp_target"};

	for(auto& object : objects) {
		int obj_handle = object_handles_.at(robot_->name() + object + suffix_);
		for(auto frame : frames) {
			int frame_id = getFrameHandle(frame);
			simxGetObjectPosition    (client_id_, obj_handle, frame_id, data,    simx_opmode_streaming);
			simxGetObjectOrientation (client_id_, obj_handle, frame_id, data+3,  simx_opmode_streaming);
		}
		simxGetObjectVelocity    (client_id_, obj_handle, data, data+3, simx_opmode_streaming);
	}

	for(auto frame : frames) {
		int frame_id = getFrameHandle(frame);
		simxGetObjectOrientation (client_id_, frame_id, -1, data,  simx_opmode_streaming);
	}

	for (size_t i = 1; i <= robot_->jointCount(); ++i) {
		int joint_handle = object_handles_.at(robot_->name() + "_joint" + std::to_string(i) + suffix_);
		simxGetJointPosition(client_id_, joint_handle, data, simx_opmode_streaming);
	}

	uint8_t ft_state;
	int obj_handle = object_handles_.at(robot_->name() + "_force_sensor" + suffix_);
	simxReadForceSensor(client_id_, obj_handle, &ft_state, data, data+3, simx_opmode_streaming);

	simxUChar* jacobian_str;
	simxInt sLength;
	simxReadStringStream(client_id_, ("Jacobian-"+robot_->name()).c_str(), &jacobian_str, &sLength, simx_opmode_streaming);
	simxReadStringStream(client_id_, ("RotMat-"+robot_->name()).c_str(), &jacobian_str, &sLength, simx_opmode_streaming);
}

int VREPDriver::getFrameHandle(phri::ReferenceFrame frame) const {
	switch(frame) {
	case phri::ReferenceFrame::TCP:
		return object_handles_.at(robot_->name() + "_tcp" + suffix_);
		break;
	case phri::ReferenceFrame::Base:
		return object_handles_.at(robot_->name() + "_base_frame" + suffix_);
		break;
	case phri::ReferenceFrame::World:
		return object_handles_.at(robot_->name() + "_world_frame" + suffix_);
		break;
	}
}

void VREPDriver::computeSpatialTransformation(phri::Matrix4dConstPtr transformation, phri::Matrix6dPtr spatial_transformation) const {
	auto& mat = *spatial_transformation;
	const auto& rot_mat = transformation->block<3,3>(0,0);
	mat.block<3,3>(3,0).setZero();
	mat.block<3,3>(0,0) = rot_mat;
	mat.block<3,3>(0,3).setZero();
	mat.block<3,3>(3,3) = rot_mat;
}

bool VREPDriver::getSimulationData(phri::ReferenceFrame frame_velocities, phri::ReferenceFrame frame_positions) {
	bool all_ok = true;

	all_ok &= readTCPPose(robot_->controlPointCurrentPose(), frame_positions);
	// all_ok &= readTCPTargetPose(robot_->controlPointTargetPose(), frame_positions);
	all_ok &= readTCPVelocity(robot_->controlPointCurrentVelocity(), frame_velocities);
	all_ok &= readTCPWrench(robot_->controlPointExternalForce());
	all_ok &= readJacobian(robot_->jacobian());
	all_ok &= readTransformationMatrix(robot_->transformationMatrix());
	all_ok &= readJointPosition(robot_->jointCurrentPosition());
	all_ok &= updateTrackedObjectsPosition();
	all_ok &= updateLaserScanners();

	computeSpatialTransformation(robot_->transformationMatrix(), robot_->spatialTransformationMatrix());

	return all_ok;
}

bool VREPDriver::sendSimulationData(phri::ReferenceFrame frame_velocities) {
	bool all_ok = true;

	// Make sure all commands are sent at the same time
	// simxPauseCommunication(client_id_, true);

	if(control_level_ == ControlLevel::TCP) {
		all_ok &= sendTCPtargetVelocity(robot_->controlPointVelocity(), frame_velocities);
	}
	else {
		// Make the TCP target tracks the TCP so that V-REP IK doesn't screw everything
		float data[3] = {0.f,0.f,0.f};
		int tcp_target_handle = object_handles_.at(robot_->name()+"_tcp_target"+suffix_);
		int tcp_handle = object_handles_.at(robot_->name()+"_tcp"+suffix_);
		all_ok &= (simxSetObjectPosition     (client_id_, tcp_target_handle, tcp_handle, data, simx_opmode_oneshot) != -1);
		all_ok &= (simxSetObjectOrientation  (client_id_, tcp_target_handle, tcp_handle, data, simx_opmode_oneshot) != -1);

		all_ok &= sendJointTargetVelocity(robot_->jointVelocity());

	}

	// simxPauseCommunication(client_id_, false);

	return all_ok;
}
