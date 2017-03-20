#include <safety_controller.h>
#include <constraints.h>

using namespace RSCL;
using namespace Constraints;
using namespace std;

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>();
	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);

	auto maximum_velocity = make_shared<double>(0.1);
	auto total_velocity = safety_controller.getTotalVelocity();
	auto velocity_constraint = make_shared<VelocityConstraint>(total_velocity, maximum_velocity);

	bool ok;
	ok = safety_controller.addConstraint("velocity limit", velocity_constraint);
	assert_msg("Step #1", ok == true);

	auto cstr = safety_controller.getConstraint("velocity limit");
	assert_msg("Step #2", cstr == velocity_constraint);

	ok = safety_controller.addConstraint("velocity limit", velocity_constraint);
	assert_msg("Step #3", ok == false);

	ok = safety_controller.addConstraint("velocity limit", velocity_constraint, true);
	assert_msg("Step #4", ok == true);

	ok = safety_controller.removeConstraint("velocity limit");
	assert_msg("Step #5", ok == true);

	ok = safety_controller.removeConstraint("velocity limit");
	assert_msg("Step #6", ok == false);

	cstr = safety_controller.getConstraint("velocity limit");
	assert_msg("Step #7", static_cast<bool>(cstr) == false);

	return 0;
}
