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

namespace phri {

/***        Robot::JointData        ***/
Robot::JointData::JointData(size_t joint_count) {
    resize(joint_count);
}
void Robot::JointData::resize(size_t joint_count) {
    position().resize(joint_count);
    velocity().resize(joint_count);
    acceleration().resize(joint_count);
    force().resize(joint_count);

    position().setZero();
    velocity().setZero();
    acceleration().setZero();
    force().setZero();
}

Eigen::VectorXd& Robot::JointData::position() {
    return position_;
}
const Eigen::VectorXd& Robot::JointData::position() const {
    return position_;
}

Eigen::VectorXd& Robot::JointData::velocity() {
    return velocity_;
}
const Eigen::VectorXd& Robot::JointData::velocity() const {
    return velocity_;
}

Eigen::VectorXd& Robot::JointData::acceleration() {
    return acceleration_;
}
const Eigen::VectorXd& Robot::JointData::acceleration() const {
    return acceleration_;
}

Eigen::VectorXd& Robot::JointData::force() {
    return force_;
}
const Eigen::VectorXd& Robot::JointData::force() const {
    return force_;
}

/***        Robot::JointLimits      ***/
Robot::JointLimits::JointLimits(size_t joint_count) {
    resize(joint_count);
}

void Robot::JointLimits::resize(size_t joint_count) {
    min_position_.resize(joint_count);
    max_position_.resize(joint_count);
    max_velocity_.resize(joint_count);
    max_acceleration_.resize(joint_count);
    max_force_.resize(joint_count);

    min_position_.setConstant(-std::numeric_limits<double>::infinity());
    max_position_.setConstant(std::numeric_limits<double>::infinity());
    max_velocity_.setConstant(std::numeric_limits<double>::infinity());
    max_acceleration_.setConstant(std::numeric_limits<double>::infinity());
    max_force_.setConstant(std::numeric_limits<double>::infinity());
}

Eigen::Ref<const Eigen::VectorXd> Robot::JointLimits::minPosition() const {
    return min_position_ * safety_factor;
}
Eigen::Ref<const Eigen::VectorXd> Robot::JointLimits::maxPosition() const {
    return max_position_ * safety_factor;
}
Eigen::Ref<const Eigen::VectorXd> Robot::JointLimits::maxVelocity() const {
    return max_velocity_ * safety_factor;
}
Eigen::Ref<const Eigen::VectorXd> Robot::JointLimits::maxAcceleration() const {
    return max_acceleration_ * safety_factor;
}
Eigen::Ref<const Eigen::VectorXd> Robot::JointLimits::maxForce() const {
    return max_force_ * safety_factor;
}

/***        Robot::Joints       ***/
Robot::Joints::Joints(size_t joint_count) {
    resize(joint_count);
}

//! \brief Call JointData::resize() on Joints::state, Joints::target and
//! Joints::command and JointLimits::resize() on limits
void Robot::Joints::resize(size_t joint_count) {
    state_.resize(joint_count);
    target_.resize(joint_count);
    command_.resize(joint_count);
    limits_.resize(joint_count);
}

const Robot::JointData& Robot::Joints::state() const {
    return state_;
}

Robot::JointData& Robot::Joints::target() {
    return target_;
}

const Robot::JointData& Robot::Joints::target() const {
    return target_;
}

const Robot::JointData& Robot::Joints::command() const {
    return command_;
}

const Robot::JointLimits& Robot::Joints::limits() const {
    return limits_;
}

/***        Robot::TaskData      ***/
Robot::TaskData::TaskData(spatial::Frame frame)
    : position_{spatial::Position::Zero(frame)},
      velocity_{spatial::Velocity(frame)},
      acceleration_{spatial::Acceleration(frame)},
      wrench_{spatial::Force(frame)} {
}

spatial::Position& Robot::TaskData::position() {
    return position_;
}

const spatial::Position& Robot::TaskData::position() const {
    return position_;
}

spatial::Velocity& Robot::TaskData::velocity() {
    return velocity_;
}

const spatial::Velocity& Robot::TaskData::velocity() const {
    return velocity_;
}

spatial::Acceleration& Robot::TaskData::acceleration() {
    return acceleration_;
}

const spatial::Acceleration& Robot::TaskData::acceleration() const {
    return acceleration_;
}

spatial::Force& Robot::TaskData::force() {
    return wrench_;
}

const spatial::Force& Robot::TaskData::force() const {
    return wrench_;
}

/***        Robot::TaskLimits      ***/
Robot::TaskLimits::TaskLimits(spatial::Frame frame)
    : min_position_{frame},
      max_position_{frame},
      max_velocity_{frame},
      max_acceleration_{frame},
      max_force_{frame} {
    min_position_.linear().setConstant(
        -std::numeric_limits<double>::infinity());
    max_position_.linear().setConstant(std::numeric_limits<double>::infinity());
    max_velocity_.setConstant(std::numeric_limits<double>::infinity());
    max_acceleration_.setConstant(std::numeric_limits<double>::infinity());
    max_force_.setConstant(std::numeric_limits<double>::infinity());
}

spatial::Position& Robot::TaskLimits::minPosition() {
    return min_position_;
}

const spatial::Position& Robot::TaskLimits::minPosition() const {
    return min_position_;
}

spatial::Position& Robot::TaskLimits::maxPosition() {
    return max_position_;
}

const spatial::Position& Robot::TaskLimits::maxPosition() const {
    return max_position_;
}

spatial::Velocity& Robot::TaskLimits::maxVelocity() {
    return max_velocity_;
}

const spatial::Velocity& Robot::TaskLimits::maxVelocity() const {
    return max_velocity_;
}

spatial::Acceleration& Robot::TaskLimits::maxAcceleration() {
    return max_acceleration_;
}

const spatial::Acceleration& Robot::TaskLimits::maxAcceleration() const {
    return max_acceleration_;
}

spatial::Force& Robot::TaskLimits::maxForce() {
    return max_force_;
}

const spatial::Force& Robot::TaskLimits::maxForce() const {
    return max_force_;
}

/***        Robot::Task     ***/
Robot::Task::Task(spatial::Frame frame)
    : state_{frame}, target_{frame}, command_{frame}, limits_{frame} {
}

const Robot::TaskData& Robot::Task::state() const {
    return state_;
}

const Robot::TaskData& Robot::Task::target() const {
    return target_;
}

Robot::TaskData& Robot::Task::target() {
    return target_;
}

const Robot::TaskData& Robot::Task::command() const {
    return command_;
}

const Robot::TaskLimits& Robot::Task::limits() const {
    return limits_;
}

/***        Robot::ControlData      ***/
Robot::ControlData::ControlData(spatial::Frame frame, spatial::Frame parent)
    : task_{frame}, transformation_{frame, parent} {
}

Robot::ControlData::ControlData(spatial::Frame frame, spatial::Frame parent,
                                size_t joint_count)
    : ControlData{frame, parent} {
    resize(joint_count);
}

void Robot::ControlData::resize(size_t joint_count) {
    joints_.resize(joint_count);
    jacobian_.resize(6, joint_count);
    jacobian_inverse_.resize(joint_count, 6);
}

Robot::ControlData::Joints& Robot::ControlData::joints() {
    return joints_;
}

const Robot::ControlData::Joints& Robot::ControlData::joints() const {
    return joints_;
}

Robot::ControlData::Task& Robot::ControlData::task() {
    return task_;
}

const Robot::ControlData::Task& Robot::ControlData::task() const {
    return task_;
}

const double& Robot::ControlData::scalingFactor() const {
    return scaling_factor_;
}

const double& Robot::ControlData::timeStep() const {
    return time_step_;
}

const Eigen::MatrixXd& Robot::ControlData::jacobian() const {
    return jacobian_;
}

const Eigen::MatrixXd& Robot::ControlData::jacobianInverse() const {
    return jacobian_inverse_;
}

const spatial::Transformation& Robot::ControlData::transformation() const {
    return transformation_;
}

/***        ControlData::Joints     ***/
Robot::ControlData::Joints::Joints(size_t joint_count) {
    resize(joint_count);
}

void Robot::ControlData::Joints::resize(size_t joint_count) {
    damping_.resize(joint_count);
    velocity_sum_.resize(joint_count);
    force_sum_.resize(joint_count);
    velocity_command_.resize(joint_count);
    total_velocity_.resize(joint_count);
    total_force_.resize(joint_count);

    damping_.setConstant(std::numeric_limits<double>::max());
    velocity_sum_.setZero();
    force_sum_.setZero();
    velocity_command_.setZero();
    total_velocity_.setZero();
    total_force_.setZero();
}

Eigen::VectorXd& Robot::ControlData::Joints::damping() {
    return damping_;
}
const Eigen::VectorXd& Robot::ControlData::Joints::damping() const {
    return damping_;
}

const Eigen::VectorXd& Robot::ControlData::Joints::velocitySum() const {
    return velocity_sum_;
}

const Eigen::VectorXd& Robot::ControlData::Joints::forceSum() const {
    return force_sum_;
}

const Eigen::VectorXd& Robot::ControlData::Joints::velocityCommand() const {
    return velocity_command_;
}

const Eigen::VectorXd& Robot::ControlData::Joints::totalVelocity() const {
    return total_velocity_;
}

const Eigen::VectorXd& Robot::ControlData::Joints::totalForce() const {
    return total_force_;
}

/***        ControlData::Task     ***/
Robot::ControlData::Task::Task(spatial::Frame frame)
    : damping_{spatial::Damping::Zero(frame)},
      velocity_sum_{spatial::Velocity::Zero(frame)},
      force_sum_{spatial::Force::Zero(frame)},
      velocity_command_{spatial::Velocity::Zero(frame)},
      total_velocity_{spatial::Velocity::Zero(frame)},
      total_force_{spatial::Force::Zero(frame)} {
    damping_.diagonal().setConstant(std::numeric_limits<double>::max());
    velocity_sum_.setZero();
    force_sum_.setZero();
    velocity_command_.setZero();
    total_velocity_.setZero();
    total_force_.setZero();
}

spatial::Damping& Robot::ControlData::Task::damping() {
    return damping_;
}
const spatial::Damping& Robot::ControlData::Task::damping() const {
    return damping_;
}

const spatial::Velocity& Robot::ControlData::Task::velocitySum() const {
    return velocity_sum_;
}

const spatial::Force& Robot::ControlData::Task::forceSum() const {
    return force_sum_;
}

const spatial::Velocity& Robot::ControlData::Task::velocityCommand() const {
    return velocity_command_;
}

const spatial::Velocity& Robot::ControlData::Task::totalVelocity() const {
    return total_velocity_;
}

const spatial::Force& Robot::ControlData::Task::totalForce() const {
    return total_force_;
}

/***    Robot       ***/
Robot::Robot(spatial::Frame control_point_frame,
             spatial::Frame control_point_parent_frame)
    : control_point_frame_{control_point_frame},
      control_point_parent_frame_{control_point_parent_frame},
      task_{spatial::Frame::Ref(control_point_frame_)},
      control_{spatial::Frame::Ref(control_point_frame_),
               spatial::Frame::Ref(control_point_parent_frame_)} {
}

Robot::Robot(spatial::Frame control_point_frame,
             spatial::Frame control_point_parent_frame, const std::string& name,
             size_t joint_count)
    : Robot{control_point_frame, control_point_parent_frame} {
    create(name, joint_count);
}

Robot::Robot(const YAML::Node& configuration)
    : Robot{spatial::Frame::Unknown(), spatial::Frame::Unknown()} {
    create(configuration);
}

void Robot::create(const std::string& name, size_t joint_count) {
    name_ = name;
    joint_count_ = joint_count;
    joints().resize(joint_count_);
    control().resize(joint_count_);
}

void Robot::create(const std::string& name, size_t joint_count,
                   spatial::Frame control_point_frame,
                   spatial::Frame control_point_parent_frame) {
    create(name, joint_count);
    control_point_frame_ = control_point_frame;
    control_point_parent_frame_ = control_point_parent_frame;
}

void Robot::create(const YAML::Node& configuration) {
    const auto& robot = configuration["robot"];
    std::string name;
    size_t joint_count;
    auto frame = spatial::Frame::Unknown();
    auto parent = spatial::Frame::Unknown();
    if (robot) {
        try {
            name = robot["name"].as<std::string>();
        } catch (...) {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "You must provide a 'name' field in the robot configuration."));
        }
        try {
            joint_count = robot["joint_count"].as<size_t>();
        } catch (...) {
            throw std::runtime_error(
                OPEN_PHRI_ERROR("You must provide a 'joint_count' field in the "
                                "robot configuration."));
        }
        try {
            auto control_point = robot["control_point"];
            frame = spatial::Frame::getAndSave(
                control_point["frame"].as<std::string>());
            parent = spatial::Frame::getAndSave(
                control_point["parent"].as<std::string>());
        } catch (...) {
            throw std::runtime_error(OPEN_PHRI_ERROR(
                "You must provide a 'control_point' field in the "
                "robot configuration."));
        }

        create(name, joint_count, frame, parent);
    } else {
        throw std::runtime_error(OPEN_PHRI_ERROR(
            "The configuration file doesn't include a 'robot' field."));
    }
}

const std::string& Robot::name() const {
    return name_;
}

size_t Robot::jointCount() const {
    return joint_count_;
}

spatial::Frame Robot::controlPointFrame() const {
    return control_point_frame_;
}

spatial::Frame Robot::controlPointParentFrame() const {
    return control_point_parent_frame_;
}

//! \brief The robot's joints data
Robot::Joints& Robot::joints() {
    return joints_;
}

const Robot::Joints& Robot::joints() const {
    return joints_;
}

//! \brief The robot's task data
Robot::Task& Robot::task() {
    return task_;
}
const Robot::Task& Robot::task() const {
    return task_;
}

//! \brief Control related data
Robot::ControlData& Robot::control() {
    return control_;
}
const Robot::ControlData& Robot::control() const {
    return control_;
}

} // namespace phri
