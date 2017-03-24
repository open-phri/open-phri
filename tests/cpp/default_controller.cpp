#include <safety_controller.h>

using namespace RSCL;
using namespace std;

int main(int argc, char const *argv[]) {

	auto damping_matrix = make_shared<Matrix6d>(Matrix6d::Identity());

	auto safety_controller = SafetyController(damping_matrix);
	safety_controller.setVerbose(true);
	auto tcp_velocity = safety_controller.getTCPVelocity();

	safety_controller.updateTCPVelocity();

	assert(tcp_velocity->isZero());

	return 0;
}
