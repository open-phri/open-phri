#include <OpenPHRI/torque_generators/torque_proxy.h>

using namespace OpenPHRI;

TorqueProxy::TorqueProxy(VectorXdConstPtr torque) :
	torque_(torque)
{
}

VectorXd TorqueProxy::compute() {
	return *torque_;
}
