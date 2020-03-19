#include "utils.h"

bool isLessOrEqual(Eigen::VectorXd v1, Eigen::VectorXd v2) {
    bool ok = true;
    for (size_t i = 0; i < v1.size(); ++i) {
        ok &= std::abs(v1(i)) <= std::abs(v2(i));
    }
    return ok;
}
