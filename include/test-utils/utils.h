#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/dummy_driver.h>
#include <utility>

struct TestData {
    TestData(double time_step = 0.001)
        : robot{phri::Robot{spatial::Frame::getAndSave("tcp"),
                            spatial::Frame::getAndSave("base"), "rob", 7}},
          model{phri::RobotModel{robot, "robot_models/kuka_lwr4.yaml",
                                 "end-effector"}},
          driver{phri::DummyDriver{robot, time_step}} {
    }

    phri::Robot robot;
    phri::RobotModel model;
    phri::DummyDriver driver;
};

bool isLessOrEqual(Eigen::VectorXd v1, Eigen::VectorXd v2);