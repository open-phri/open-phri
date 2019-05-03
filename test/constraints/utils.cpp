#include "utils.h"

bool isLessOrEqual(phri::VectorXd v1, phri::VectorXd v2) {
    bool ok = true;
    for (size_t i = 0; i < v1.size(); ++i) {
        ok &= std::abs(v1(i)) <= std::abs(v2(i));
    }
    return ok;
}

bool isClose(double v1, double v2, double eps) {
    return std::abs(v1 - v2) < eps;
}

double power(const phri::Twist& velocity, const phri::Wrench& force) {
    return force.vector().dot(velocity.vector());
}
