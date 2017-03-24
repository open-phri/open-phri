#include <force_proxy.h>

using namespace RSCL;

ForceProxy::ForceProxy(Vector6dConstPtr force) :
	force_(force)
{
}

Vector6d ForceProxy::compute() {
	return *force_;
}
