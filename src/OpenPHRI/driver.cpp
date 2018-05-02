#include <OpenPHRI/drivers/driver.h>

using namespace phri;

std::map<std::string, DriverFactory::create_method_t> DriverFactory::create_methods_;

Driver::Driver(RobotPtr robot) :
	robot_(robot)
{
}

Driver::~Driver() = default;

double Driver::getSampleTime() const {
	return sample_time_;
}
