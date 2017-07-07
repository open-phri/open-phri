#include <RSCL/velocity_generators/velocity_proxy.h>

using namespace RSCL;

VelocityProxy::VelocityProxy(Vector6dConstPtr velocity) :
	velocity_(velocity)
{
}

VelocityProxy::VelocityProxy(Vector6dConstPtr velocity, Matrix6dConstPtr spatial_transformation) :
	VelocityProxy(velocity)
{
	spatial_transformation_ = spatial_transformation;
}

Vector6d VelocityProxy::compute() {
	if(static_cast<bool>(spatial_transformation_)) {
		return spatial_transformation_->transpose() * *velocity_; // TODO find a better way transform data between frames
	}
	return *velocity_;
}
