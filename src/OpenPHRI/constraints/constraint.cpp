#include <OpenPHRI/constraints/constraint.h>

using namespace OpenPHRI;

void Constraint::setRobot(RobotConstPtr robot) {
	robot_ = robot;
}

double Constraint::operator()() {
	return compute();
}
