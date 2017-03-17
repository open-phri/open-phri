#include <safety_controller.h>
#include <constraints.h>

using namespace RSCL;
using namespace Constraints;
using namespace std;

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>();
	auto safety_controller = SafetyController(damping_matrix);

	auto maximum_velocity = make_shared<double>(0.1);
	auto total_velocity = safety_controller.getTotalVelocity();
	auto velocity_constraint = make_shared<VelocityConstraint>(total_velocity, maximum_velocity);

	safety_controller.addConstraint("velocity limit", velocity_constraint);

	bool ok;
	ok = safety_controller.removeConstraint("velocity limit");
	assert(("Step #1", ok == true));

	ok = safety_controller.removeConstraint("velocity limit");
	assert(("Step #2", ok == false));

	return 0;
}
