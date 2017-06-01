#include <RSCL/torque_generators/torque_proxy.h>

using namespace RSCL;

TorqueProxy::TorqueProxy(VectorXdConstPtr torque) :
	torque_(torque)
{
}

VectorXd TorqueProxy::compute() {
	return *torque_;
}
