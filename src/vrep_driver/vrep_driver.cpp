#include <vrep_driver/vrep_driver.h>

#include <iostream>
#include <stdexcept>

#include <extApi.h>
#include <v_repConst.h>

using namespace vrep;
using namespace std;

VREPDriver::VREPDriver(
	double sample_time,
	const std::string& prefix,
	const std::string& suffix,
	const std::string& ip,
	int port) :
	sample_time_(sample_time),
	sync_mode_(false),
	prefix_(prefix),
	suffix_(suffix)
{
	init(ip, port);
}

VREPDriver::VREPDriver(
	double sample_time,
	int client_id,
	const std::string& prefix,
	const std::string& suffix) :
	sample_time_(sample_time),
	sync_mode_(false),
	prefix_(prefix),
	suffix_(suffix)
{
	init(client_id);
}

VREPDriver::~VREPDriver() {
	enableSynchonous(false);
}

void VREPDriver::init(const std::string& ip, int port) {
	client_id_ = simxStart((simxChar*)ip.c_str(), port, 1, 1, 2000, int(sample_time_*1000));

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

	bool all_ok = true;
	all_ok &= getObjectHandles();
	all_ok &= startStreaming();
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

bool VREPDriver::readTCPPose(RSCL::Vector6dPtr pose, ReferenceFrame frame) const {
	bool all_ok = true;
	float data[6];

	int object_handle = object_handles_.at(prefix_ + "tcp*" + suffix_);
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

bool VREPDriver::readTCPVelocity(RSCL::Vector6dPtr velocity, ReferenceFrame frame) const {
	using namespace Eigen;

	bool all_ok = true;
	float data[6], angles[3];

	int object_handle = object_handles_.at(prefix_ + "tcp*" + suffix_);
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

bool VREPDriver::readTCPTargetPose(RSCL::Vector6dPtr pose, ReferenceFrame frame) const {
	bool all_ok = true;
	float data[6];

	int object_handle = object_handles_.at(prefix_ + "tcp_target" + suffix_);
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

bool VREPDriver::sendTCPtargetVelocity(RSCL::Vector6dConstPtr velocity, ReferenceFrame frame) const {
	bool all_ok = true;

	auto pose = make_shared<RSCL::Vector6d>();
	all_ok &= readTCPTargetPose(pose, frame);
	*pose += *velocity * sample_time_;

	float data[6];
	double* pose_data = pose->data();
	for (size_t i = 0; all_ok and i < 6; ++i) {
		data[i] = pose_data[i];
	}
	int object_handle = object_handles_.at(prefix_ + "tcp_target" + suffix_);
	int frame_id = getFrameHandle(frame);
	all_ok &= (simxSetObjectPosition    (client_id_, object_handle, frame_id, data,    simx_opmode_oneshot) == simx_return_ok);
	all_ok &= (simxSetObjectOrientation (client_id_, object_handle, frame_id, data+3,  simx_opmode_oneshot) == simx_return_ok);

	return all_ok;
}

bool VREPDriver::readTCPWrench(RSCL::Vector6dPtr wrench) const {
	bool all_ok = true;
	float data[6];
	uint8_t ft_state;
	string obj_name = prefix_ + "force_sensor" + suffix_;

	all_ok &= (simxReadForceSensor(client_id_, object_handles_.at(obj_name), &ft_state, data, data+3, simx_opmode_buffer) == simx_return_ok);

	if(all_ok) {
		double* wrench_data = wrench->data();
		for (size_t i = 0; all_ok and i < 6; ++i) {
			wrench_data[i] = data[i];
		}
	}

	return all_ok;
}

bool VREPDriver::getObjectHandles() {
	bool all_ok = true;

	auto getHandle =
		[this](const std::string& name) -> bool {
			string obj_name = prefix_ + name + suffix_;
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

	return all_ok;
}

bool VREPDriver::startStreaming() const {
	bool all_ok = true;
	float data[6];

	ReferenceFrame frames[] = {ReferenceFrame::TCP, ReferenceFrame::Base, ReferenceFrame::World};
	string objects[] = {"tcp", "tcp_target"};

	for(auto& object : objects) {
		int obj_handle = object_handles_.at(prefix_ + object + suffix_);
		for(auto frame : frames) {
			int frame_id = getFrameHandle(frame);
			all_ok &= (simxGetObjectPosition    (client_id_, obj_handle, frame_id, data,    simx_opmode_streaming) == simx_return_ok);
			all_ok &= (simxGetObjectOrientation (client_id_, obj_handle, frame_id, data+3,  simx_opmode_streaming) == simx_return_ok);
		}
		all_ok &= (simxGetObjectVelocity    (client_id_, obj_handle, data, data+3, simx_opmode_streaming) == simx_return_ok);
	}

	uint8_t ft_state;
	int obj_handle = object_handles_.at(prefix_ + "force_sensor" + suffix_);
	all_ok &= (simxReadForceSensor(client_id_, obj_handle, &ft_state, data, data+3, simx_opmode_streaming) == simx_return_ok);

	return all_ok;
}

int VREPDriver::getFrameHandle(ReferenceFrame frame) const {
	// for(auto obj : object_handles_) {
	//  cout << obj.first << ": " << obj.second << endl;
	// }

	switch(frame) {
	case ReferenceFrame::TCP:
		return object_handles_.at(prefix_ + "tcp" + suffix_);
		break;
	case ReferenceFrame::Base:
		return object_handles_.at(prefix_ + "base_frame" + suffix_);
		break;
	case ReferenceFrame::World:
		return object_handles_.at(prefix_ + "world_frame" + suffix_);
		break;
	}
}
