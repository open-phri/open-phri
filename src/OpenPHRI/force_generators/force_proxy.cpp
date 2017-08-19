#include <OpenPHRI/force_generators/force_proxy.h>

using namespace OpenPHRI;

ForceProxy::ForceProxy(Vector6dConstPtr force, ReferenceFrame frame) :
	ForceGenerator(frame),
	force_(force)
{
}

Vector6d ForceProxy::compute() {
	return transform(*force_);
}
