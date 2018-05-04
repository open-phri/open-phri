#include <OpenPHRI/drivers/driver.h>

using namespace phri;

std::map<std::string, DriverFactory::create_method_t> DriverFactory::create_methods_;

Driver::Driver(RobotPtr robot) :
	robot_(robot)
{
}

Driver::~Driver() = default;

bool Driver::init(double timeout) {
	*robot_->jointTargetPosition() = *robot_->jointCurrentPosition();
	return true;
}

double Driver::getSampleTime() const {
	return sample_time_;
}
