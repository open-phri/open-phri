#pragma once

#include <OpenPHRI/OpenPHRI.h>

bool isLessOrEqual(Eigen::VectorXd v1, Eigen::VectorXd v2);
bool isClose(double v1, double v2, double eps = 1e-3);
double power(const spatial::Velocity& velocity, const phri::Wrench& force);