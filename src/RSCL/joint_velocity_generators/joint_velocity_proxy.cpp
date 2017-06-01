#include <RSCL/joint_velocity_generators/joint_velocity_proxy.h>

using namespace RSCL;

JointVelocityProxy::JointVelocityProxy(VectorXdConstPtr joint_velocity) :
	joint_velocity_(joint_velocity)
{
}

VectorXd JointVelocityProxy::compute() {
	return *joint_velocity_;
}
