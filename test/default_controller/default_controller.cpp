#undef NDEBUG

#include <RSCL/RSCL.h>

using namespace RSCL;
using namespace std;

int main(int argc, char const *argv[]) {

	auto robot = make_shared<Robot>(
		"rob",  // Robot's name
		7);     // Robot's joint count

	auto safety_controller = SafetyController(robot);
	safety_controller.setVerbose(true);

	safety_controller.compute();

	assert(robot->controlPointVelocity()->isZero());

	return 0;
}
