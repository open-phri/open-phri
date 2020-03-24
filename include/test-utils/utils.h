#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/dummy_driver.h>
#include <utility>

struct TestData {
    TestData(double time_step = 0.001);

    phri::Robot robot;
    phri::RobotModel model;
    phri::DummyDriver driver;
};

bool isLessOrEqual(Eigen::VectorXd v1, Eigen::VectorXd v2);