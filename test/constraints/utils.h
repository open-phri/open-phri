#pragma once

#include <OpenPHRI/OpenPHRI.h>

bool isLessOrEqual(phri::VectorXd v1, phri::VectorXd v2);
bool isClose(double v1, double v2, double eps = 1e-3);
double power(const phri::Twist& velocity, const phri::Wrench& force);