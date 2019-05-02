/*      File: robot.cpp
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2017 -
 * Benjamin Navarro (LIRMM). All Right reserved.
 *
 *       This software is free software: you can redistribute it and/or modify
 *       it under the terms of the LGPL license as published by
 *       the Free Software Foundation, either version 3
 *       of the License, or (at your option) any later version.
 *       This software is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *       LGPL License for more details.
 *
 *       You should have received a copy of the GNU Lesser General Public
 * License version 3 and the General Public License version 3 along with this
 * program. If not, see <http://www.gnu.org/licenses/>.
 */

#include <OpenPHRI/robot.h>
#include <OpenPHRI/utilities/exceptions.h>

#include <yaml-cpp/yaml.h>

#include <iostream>

using namespace phri;

Robot::Robot(const std::string& name, size_t joint_count) {
    create(name, joint_count);
}

Robot::Robot(const YAML::Node& configuration) {
    create(configuration);
}

void Robot::create(const std::string& name, size_t joint_count) {
    name_ = name;
    joint_count_ = joint_count;
    create();
}

void Robot::create(const YAML::Node& configuration) {
    const auto& robot = configuration["robot"];
    if (robot) {
        try {
            name_ = robot["name"].as<std::string>();
        } catch (...) {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "You must provide a 'name' field in the robot configuration."));
        }
        try {
            joint_count_ = robot["joint_count"].as<size_t>();
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'joint_count' field in the "
                                "robot configuration."));
        }

        create();
    } else {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The configuration file doesn't include a 'robot' field."));
    }
}

void Robot::create() {
    joint_damping_matrix_ = std::make_shared<VectorXd>();
    control_point_damping_matrix_ =
        std::make_shared<Vector6d>(Vector6d::Ones());

    joint_velocity_ = std::make_shared<Eigen::VectorXd>();
    joint_velocity_sum_ = std::make_shared<Eigen::VectorXd>();
    joint_torque_sum_ = std::make_shared<Eigen::VectorXd>();
    joint_velocity_command_ = std::make_shared<Eigen::VectorXd>();
    joint_total_velocity_ = std::make_shared<Eigen::VectorXd>();
    joint_total_torque_ = std::make_shared<Eigen::VectorXd>();

    joint_current_position_ = std::make_shared<Eigen::VectorXd>();
    joint_target_position_ = std::make_shared<Eigen::VectorXd>();
    joint_external_torque_ = std::make_shared<Eigen::VectorXd>();

    control_point_velocity_ = std::make_shared<Twist>();
    control_point_velocity_sum_ = std::make_shared<Twist>();
    control_point_force_sum_ = std::make_shared<Vector6d>(Vector6d::Zero());
    control_point_velocity_command_ = std::make_shared<Twist>();
    control_point_total_velocity_ = std::make_shared<Twist>();
    control_point_total_force_ = std::make_shared<Vector6d>(Vector6d::Zero());

    control_point_current_pose_ = std::make_shared<Pose>();
    control_point_target_pose_ = std::make_shared<Pose>();
    control_point_current_velocity_ = std::make_shared<Twist>();
    control_point_current_acceleration_ = std::make_shared<Acceleration>();
    control_point_external_force_ =
        std::make_shared<Vector6d>(Vector6d::Zero());

    scaling_factor_ = std::make_shared<double>();

    jacobian_ = std::make_shared<MatrixXd>();
    jacobian_inverse_ = std::make_shared<MatrixXd>();
    transformation_matrix_ = std::make_shared<Matrix4d>(Matrix4d::Identity());
    spatial_transformation_matrix_ =
        std::make_shared<Matrix6d>(Matrix6d::Identity());

    joint_velocity_->resize(joint_count_, 1);
    joint_velocity_->setZero();
    joint_velocity_sum_->resize(joint_count_, 1);
    joint_velocity_sum_->setZero();
    joint_torque_sum_->resize(joint_count_, 1);
    joint_torque_sum_->setZero();
    joint_velocity_command_->resize(joint_count_, 1);
    joint_velocity_command_->setZero();
    joint_total_velocity_->resize(joint_count_, 1);
    joint_total_velocity_->setZero();
    joint_total_torque_->resize(joint_count_, 1);
    joint_total_torque_->setZero();

    joint_current_position_->resize(joint_count_, 1);
    joint_current_position_->setZero();
    joint_target_position_->resize(joint_count_, 1);
    joint_target_position_->setZero();
    joint_external_torque_->resize(joint_count_, 1);
    joint_external_torque_->setZero();

    jacobian_->resize(6, joint_count_);
    jacobian_->setZero();
    jacobian_inverse_->resize(joint_count_, 6);
    jacobian_inverse_->setZero();

    joint_damping_matrix_->resize(joint_count_, 1);
    joint_damping_matrix_->setOnes();
}

const std::string& Robot::name() const {
    return name_;
}

size_t Robot::jointCount() const {
    return joint_count_;
}

VectorXdPtr Robot::jointDampingMatrix() const {
    return joint_damping_matrix_;
}

Vector6dPtr Robot::controlPointDampingMatrix() const {
    return control_point_damping_matrix_;
}

VectorXdConstPtr Robot::jointVelocity() const {
    return joint_velocity_;
}

VectorXdConstPtr Robot::jointVelocitySum() const {
    return joint_velocity_sum_;
}

VectorXdConstPtr Robot::jointTorqueSum() const {
    return joint_torque_sum_;
}

VectorXdConstPtr Robot::jointVelocityCommand() const {
    return joint_velocity_command_;
}

VectorXdConstPtr Robot::jointTotalVelocity() const {
    return joint_total_velocity_;
}

VectorXdConstPtr Robot::jointTotalTorque() const {
    return joint_total_torque_;
}

VectorXdPtr Robot::jointCurrentPosition() const {
    return joint_current_position_;
}

VectorXdPtr Robot::jointTargetPosition() const {
    return joint_target_position_;
}

VectorXdPtr Robot::jointExternalTorque() const {
    return joint_external_torque_;
}

TwistConstPtr Robot::controlPointVelocity() const {
    return control_point_velocity_;
}

TwistConstPtr Robot::controlPointVelocitySum() const {
    return control_point_velocity_sum_;
}

Vector6dConstPtr Robot::controlPointForceSum() const {
    return control_point_force_sum_;
}

TwistConstPtr Robot::controlPointVelocityCommand() const {
    return control_point_velocity_command_;
}

TwistConstPtr Robot::controlPointTotalVelocity() const {
    return control_point_total_velocity_;
}

Vector6dConstPtr Robot::controlPointTotalForce() const {
    return control_point_total_force_;
}

PosePtr Robot::controlPointCurrentPose() const {
    return control_point_current_pose_;
}

PosePtr Robot::controlPointTargetPose() const {
    return control_point_target_pose_;
}

TwistPtr Robot::controlPointCurrentVelocity() const {
    return control_point_current_velocity_;
}

AccelerationPtr Robot::controlPointCurrentAcceleration() const {
    return control_point_current_acceleration_;
}

Vector6dPtr Robot::controlPointExternalForce() const {
    return control_point_external_force_;
}

doubleConstPtr Robot::scalingFactor() const {
    return scaling_factor_;
}

MatrixXdPtr Robot::jacobian() const {
    return jacobian_;
}

MatrixXdPtr Robot::jacobianInverse() const {
    return jacobian_inverse_;
}

Matrix4dPtr Robot::transformationMatrix() const {
    return transformation_matrix_;
}

Matrix6dPtr Robot::spatialTransformationMatrix() const {
    return spatial_transformation_matrix_;
}
