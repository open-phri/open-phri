#include <RSCL/constraints/constraint.h>

using namespace RSCL;

void Constraint::setRobot(RobotConstPtr robot) {
	robot_ = robot;
}

double Constraint::operator()() {
	return compute();
}
