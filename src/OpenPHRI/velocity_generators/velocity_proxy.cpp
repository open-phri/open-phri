#include <OpenPHRI/velocity_generators/velocity_proxy.h>

using namespace OpenPHRI;

VelocityProxy::VelocityProxy(Vector6dConstPtr velocity, ReferenceFrame frame) :
	VelocityGenerator(frame),
	external_velocity_(velocity)
{
}

Vector6d VelocityProxy::compute() {
	velocity_ = *external_velocity_;
	return VelocityGenerator::compute();;
}
