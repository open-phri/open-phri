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

	/***				V-REP driver				***/
	VREPDriver driver(
		SAMPLE_TIME,
		"LBR4p_");      // LBR4p_: Robot prefix

	driver.startSimulation();

	/***			Controller configuration			***/
	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity() * 100.);
	auto safety_controller = SafetyController(damping_matrix);

	auto tcp_velocity = safety_controller.getTCPVelocity();

	auto maximum_velocity = make_shared<double>(0.1);
	auto velocity_constraint = make_shared<VelocityConstraint>(maximum_velocity);

	// Objects are tracked in the TCP frame so there is no need to provide the robot position
	auto potential_field_generator = make_shared<PotentialFieldGenerator>();
	potential_field_generator->setVerbose(true);

	auto obstacle1 = make_shared<PotentialFieldObject>(
		PotentialFieldType::Repulsive,
		make_shared<double>(10.),   // gain
		make_shared<double>(0.2),   // threshold distance
		driver.trackObjectPosition("obstacle1", ReferenceFrame::TCP));

	auto obstacle2 = make_shared<PotentialFieldObject>(
		PotentialFieldType::Repulsive,
		make_shared<double>(10.),   // gain
		make_shared<double>(0.2),   // threshold distance
		driver.trackObjectPosition("obstacle2", ReferenceFrame::TCP));

	auto target = make_shared<PotentialFieldObject>(
		PotentialFieldType::Attractive,
		make_shared<double>(10.),  // gain
		make_shared<double>(std::numeric_limits<double>::infinity()),   // threshold distance
		driver.trackObjectPosition("target", ReferenceFrame::TCP));

	potential_field_generator->add("obstacle1", obstacle1);
	potential_field_generator->add("obstacle2", obstacle2);
	potential_field_generator->add("target", target);

	safety_controller.addConstraint(
		"velocity constraint",
		velocity_constraint);

	safety_controller.addForceGenerator(
		"potential field",
		potential_field_generator);

	signal(SIGINT, sigint_handler);

	driver.enableSynchonous(true);

	cout << "Starting main loop" << endl;
	while(not _stop) {
		driver.updateTrackedObjectsPosition();
		safety_controller.updateTCPVelocity();
		driver.sendTCPtargetVelocity(tcp_velocity, ReferenceFrame::TCP);

		driver.nextStep();
	}

	driver.enableSynchonous(false);
	driver.stopSimulation();

	return 0;
}
