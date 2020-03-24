#include "utils.h"

TestData::TestData(double time_step)
    : robot{spatial::Frame::getAndSave("tcp"),
            spatial::Frame::getAndSave("base"), "rob", 7},
      model{robot, "robot_models/kuka_lwr4.yaml", "end-effector"},
      driver{robot, time_step} {
}

bool isLessOrEqual(Eigen::VectorXd v1, Eigen::VectorXd v2) {
    bool ok = true;
    for (size_t i = 0; i < v1.size(); ++i) {
        ok &= std::abs(v1(i)) <= std::abs(v2(i));
    }
    return ok;
}
