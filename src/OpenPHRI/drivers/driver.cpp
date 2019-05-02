#include <OpenPHRI/drivers/driver.h>

using namespace phri;

Driver::Driver(Robot& robot, double sample_time) : robot_(robot) {
    robot_.control.time_step = sample_time;
}

Driver::~Driver() = default;

bool Driver::init(double timeout) {
    double waited_for = 0.;

    while (not read() and waited_for < timeout) {
        waited_for += getSampleTime();
    }

    if (waited_for < timeout) {
        robot_.joints.command.position = robot_.joints.state.position;
        return true;
    }

    return false;
}

bool Driver::sync() {
    return true;
}

double Driver::getSampleTime() const {
    return robot_.control.time_step;
}
