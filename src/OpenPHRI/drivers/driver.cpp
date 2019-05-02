#include <OpenPHRI/drivers/driver.h>

using namespace phri;

Driver::Driver(RobotPtr robot, double sample_time)
    : robot_(robot), sample_time_(sample_time) {
}

Driver::~Driver() = default;

bool Driver::init(double timeout) {
    double waited_for = 0.;

    while (not read() and waited_for < timeout) {
        waited_for += getSampleTime();
    }

    if (waited_for < timeout) {
        *robot_->jointTargetPosition() = *robot_->jointCurrentPosition();
        return true;
    }

    return false;
}

bool Driver::sync() {
    return true;
}

double Driver::getSampleTime() const {
    return sample_time_;
}
