#pragma once

#include <string>
#include <unordered_map>

#include <definitions.h>

namespace vrep {

enum class ReferenceFrame {
	TCP,
	Base,
	World
};

class VREPDriver {
public:
	VREPDriver(
		double sample_time,
		const std::string& prefix = "",
		const std::string& suffix = "",
		const std::string& ip = "127.0.0.1",
		int port = 19997);

	VREPDriver(
		double sample_time,
		int client_id,
		const std::string& prefix = "",
		const std::string& suffix = "");

	~VREPDriver();

	int getClientID() const;
	bool checkConnection() const;
	bool enableSynchonous(bool state);
	bool nextStep() const;

	void startSimulation() const;
	void stopSimulation() const;
	void pauseSimulation() const;

	bool readTCPPose(RSCL::Vector6dPtr pose, ReferenceFrame frame) const;
	bool readTCPVelocity(RSCL::Vector6dPtr velocity, ReferenceFrame frame) const;

	bool readTCPTargetPose(RSCL::Vector6dPtr pose, ReferenceFrame frame) const;
	bool sendTCPtargetVelocity(RSCL::Vector6dConstPtr velocity, ReferenceFrame frame) const;

	bool readTCPWrench(RSCL::Vector6dPtr wrench) const;

private:
	void init(const std::string& ip, int port);
	void init(int client_id);
	bool getObjectHandles();
	bool startStreaming() const;
	int getFrameHandle(ReferenceFrame frame) const;

	int client_id_;
	double sample_time_;
	bool sync_mode_;

	std::string prefix_;
	std::string suffix_;

	std::unordered_map<std::string, int> object_handles_;

};

}
