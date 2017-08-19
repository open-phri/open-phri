#include <OpenPHRI/force_generators/external_force.h>

using namespace OpenPHRI;

ExternalForce::ExternalForce(RobotConstPtr robot) :
	ForceGenerator(ReferenceFrame::TCP),
	external_force_(robot->controlPointExternalForce())
{
}

Vector6d ExternalForce::compute() {
	force_ = *external_force_;
	return ForceGenerator::compute();
}
