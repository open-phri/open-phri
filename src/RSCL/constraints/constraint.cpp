#include <RSCL/constraints/constraint.h>

using namespace RSCL;

void Constraint::setRobot(RobotConstPtr robot) {
	robot_ = robot;
}
