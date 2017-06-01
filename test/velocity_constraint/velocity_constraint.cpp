#undef NDEBUG

#include <RSCL/RSCL.h>

using namespace RSCL;
using namespace std;

bool isClose(double v1, double v2, double eps = 1e-3) {
	return std::abs(v1-v2) < eps;
}

int main(int argc, char const *argv[]) {

	auto robot = make_shared<Robot>(
		"rob",  // Robot's name
		7);     // Robot's joint count

	auto safety_controller = SafetyController(robot);
	safety_controller.setVerbose(true);

	auto maximum_velocity = make_shared<double>(0.5);
	auto velocity_constraint = make_shared<VelocityConstraint>(maximum_velocity);

	auto constant_vel = make_shared<Vector6d>(Vector6d::Zero());
	auto constant_velocity_generator = make_shared<VelocityProxy>(constant_vel);

	safety_controller.add("velocity constraint", velocity_constraint);
	safety_controller.add("vel proxy", constant_velocity_generator);

	auto& translation_velocity = robot->controlPointVelocity()->block<3,1>(0,0);

	// Step #1 : no velocity
	safety_controller.compute();

	assert_msg("Step #1", robot->controlPointVelocity()->isZero());

	// Step #2 : velocity 1 axis < max
	(*constant_vel)(0) = 0.2;
	safety_controller.compute();

	assert_msg("Step #2", robot->controlPointVelocity()->isApprox(*robot->controlPointTotalVelocity()));

	// Step #3 : velocity 1 axis > max
	(*constant_vel)(0) = 0.6;
	safety_controller.compute();

	assert_msg("Step #3", isClose(translation_velocity.norm(), *maximum_velocity));

	// Step #4 : velocity 3 axes < max
	(*constant_vel)(0) = 0.2;
	(*constant_vel)(1) = 0.1;
	(*constant_vel)(2) = 0.3;
	safety_controller.compute();

	assert_msg("Step #4", robot->controlPointVelocity()->isApprox(*robot->controlPointTotalVelocity()));

	// Step #5 : velocity 3 axes > max
	(*constant_vel)(0) = 0.5;
	(*constant_vel)(1) = 0.4;
	(*constant_vel)(2) = 0.6;
	safety_controller.compute();

	assert_msg("Step #5", isClose(translation_velocity.norm(), *maximum_velocity));

	// Step #6 : rotational velocity only
	constant_vel->setZero();
	(*constant_vel)(4) = 0.5;
	(*constant_vel)(4) = 0.4;
	(*constant_vel)(5) = 0.6;
	safety_controller.compute();

	assert_msg("Step #6", robot->controlPointVelocity()->isApprox(*robot->controlPointTotalVelocity()));

	return 0;
}
