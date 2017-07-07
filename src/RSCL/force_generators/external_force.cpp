#include <RSCL/force_generators/external_force.h>

using namespace RSCL;

ExternalForce::ExternalForce(RobotConstPtr robot) :
	force_(robot->controlPointExternalForce())
{
}

Vector6d ExternalForce::compute() {
	return *force_;
}
