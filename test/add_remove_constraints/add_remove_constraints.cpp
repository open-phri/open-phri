#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

using namespace OpenPHRI;
using namespace std;

int main(int argc, char const *argv[]) {

	auto robot = make_shared<Robot>(
		"rob",  // Robot's name
		7);     // Robot's joint count

	auto safety_controller = SafetyController(robot);
	safety_controller.setVerbose(true);

	auto maximum_velocity = make_shared<double>(0.1);
	auto velocity_constraint = make_shared<VelocityConstraint>(maximum_velocity);

	bool ok;
	ok = safety_controller.add("velocity limit", velocity_constraint);
	assert_msg("Step #1", ok == true);

	auto cstr = safety_controller.getConstraint("velocity limit");
	assert_msg("Step #2", cstr == velocity_constraint);

	ok = safety_controller.add("velocity limit", VelocityConstraint(maximum_velocity));
	assert_msg("Step #3", ok == false);

	ok = safety_controller.add("velocity limit", std::move(*velocity_constraint), true);
	assert_msg("Step #4", ok == true);

	ok = safety_controller.removeConstraint("velocity limit");
	assert_msg("Step #5", ok == true);

	ok = safety_controller.removeConstraint("velocity limit");
	assert_msg("Step #6", ok == false);

	cstr = safety_controller.getConstraint("velocity limit");
	assert_msg("Step #7", static_cast<bool>(cstr) == false);

	return 0;
}
