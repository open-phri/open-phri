#include <iostream>
#include <unistd.h>
#include <signal.h>

#include <RSCL/RSCL.h>
#include <vrep_driver/vrep_driver.h>

using namespace std;
using namespace RSCL;
using namespace vrep;

constexpr double SAMPLE_TIME = 0.010;

bool _stop = false;

void sigint_handler(int sig) {
	_stop = true;
}

int main(int argc, char const *argv[]) {

	/***			Controller configuration			***/
	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 100.);
	auto safety_controller = SafetyController(damping_matrix);

	auto tcp_velocity = safety_controller.getTCPVelocity();
	auto total_velocity = safety_controller.getTotalVelocity();

	auto maximum_velocity = make_shared<double>(0.1);
	auto velocity_constraint = make_shared< VelocityConstraint >(
		total_velocity,
		maximum_velocity);

	auto reference_vel = make_shared<Vector6d>();
	auto constant_vel_gen = make_shared<VelocityProxy>(reference_vel);

	auto ext_force = make_shared<Vector6d>(Vector6d::Zero());
	auto ext_force_generator = make_shared<ForceProxy>(ext_force);

	safety_controller.addConstraint(
		"velocity constraint",
		velocity_constraint);

	safety_controller.addVelocityGenerator(
		"vel proxy",
		constant_vel_gen);

	safety_controller.addForceGenerator(
		"ext force proxy",
		ext_force_generator);

	/***				V-REP driver				***/
	VREPDriver driver(
		SAMPLE_TIME,
		"LBR4p_");      // LBR4p_: Robot prefix

	driver.enableSynchonous(true);
	driver.startSimulation();

	signal(SIGINT, sigint_handler);

	while(not _stop) {
		driver.readTCPWrench(ext_force);
		safety_controller.updateTCPVelocity();
		driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame::TCP);

		driver.nextStep();
	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
