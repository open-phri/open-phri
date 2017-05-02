#include <RSCL/velocity_generators/velocity_proxy.h>

using namespace RSCL;

VelocityProxy::VelocityProxy(Vector6dConstPtr velocity) :
	velocity_(velocity)
{
}

Vector6d VelocityProxy::compute() {
	return *velocity_;
}
