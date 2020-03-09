#include <OpenPHRI/drivers/driver.h>

using namespace phri;

Driver::Driver(Robot& robot, double sample_time) : robot_(robot) {
    robot_.control().time_step_ = sample_time;
}

Driver::~Driver() = default;

bool Driver::init(double timeout) {
    double waited_for = 0.;

    while (not read() and waited_for < timeout) {
        waited_for += getTimeStep();
    }

    if (waited_for < timeout) {
        robot_.joints().command_.position() =
            robot_.joints().state().position();
        return true;
    }

    return false;
}

bool Driver::sync() {
    return true;
}

bool Driver::syncThenRead() {
    return sync() and read();
}

double Driver::getTimeStep() const {
    return robot_.control().timeStep();
}

void Driver::setTimeStep(double time_step) {
    robot_.control().time_step_ = time_step;
}

Robot::JointData& Driver::jointState() {
    return robot_.joints().state_;
}

const Robot::JointData& Driver::jointCommand() const {
    return robot_.joints().command_;
}

Robot::TaskData& Driver::taskState() {
    return robot_.task().state_;
}

const Robot::TaskData& Driver::taskCommand() const {
    return robot_.task().command_;
}
