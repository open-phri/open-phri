/*      File: safety_controller.cpp
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

#include <OpenPHRI/safety_controller.h>

#include <OpenPHRI/constraints/default_constraint.h>
#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/joint_force_generators/joint_force_generator.h>
#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/utilities/demangle.h>

#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <iostream>

using namespace phri;

#define HEAVY_PRINTING 0

SafetyController::SafetyController(Robot& robot)
    : robot_(robot), skip_jacobian_inverse_computation_(false) {
    addConstraint("default constraint", std::make_shared<DefaultConstraint>(),
                  true);
    dynamic_dls_ = false;
    lambda2_ = -1.;
    sigma_min_threshold_ = -1.;
}

SafetyController::SafetyController(Robot& robot, YAML::Node& configuration)
    : SafetyController(robot) {
    auto controller = configuration["controller"];
    if (controller) {
        dynamic_dls_ = controller["use_dynamic_dls"].as<bool>(dynamic_dls_);
        double lambda_max = controller["lambda_max"].as<double>(lambda2_);
        double sigma_min_threshold =
            controller["sigma_min_threshold"].as<double>(sigma_min_threshold_);

        if (dynamic_dls_) {
            enableDynamicDampedLeastSquares(lambda_max, sigma_min_threshold);
        } else if (lambda_max > 0.) {
            enableDampedLeastSquares(lambda_max);
        }
    }
}

void SafetyController::setVerbose(bool on) {
    constraints_.setVerbose(on, "phri::SafetyController", "Constraint");
    force_generators_.setVerbose(on, "phri::SafetyController",
                                 "ForceGenerator");
    torque_generators_.setVerbose(on, "phri::SafetyController",
                                  "JointForceGenerator");
    velocity_generators_.setVerbose(on, "phri::SafetyController",
                                    "VelocityGenerator");
    joint_velocity_generators_.setVerbose(on, "phri::SafetyController",
                                          "JointVelocityGenerator");
}

void SafetyController::skipJacobianInverseComputation(bool on) {
    skip_jacobian_inverse_computation_ = on;
}

void SafetyController::addConstraint(const std::string& name,
                                     std::shared_ptr<Constraint> constraint,
                                     bool force) {
    constraint->setRobot(&robot_);
    if (not constraints_.add(name, StorageWrapper<Constraint>{constraint, 0.},
                             force)) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Failed to add constraint " + name +
                            ". Use a different name or call "
                            "SafetyController::forceAdd instead to "
                            "replace the existing one."));
    }
}

void SafetyController::addForceGenerator(
    const std::string& name, std::shared_ptr<ForceGenerator> generator,
    bool force) {
    generator->setRobot(&robot_);
    generator->setFrame(robot_.controlPointFrame());
    if (not force_generators_.add(
            name,
            StorageWrapper<ForceGenerator>{
                generator, spatial::Force::Zero(robot_.controlPointFrame())},
            force)) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Failed to add generator " + name +
                            ". Use a different name or call "
                            "SafetyController::forceAdd instead to "
                            "replace the existing one."));
    }
}

void SafetyController::addJointForceGenerator(
    const std::string& name, std::shared_ptr<JointForceGenerator> generator,
    bool force) {
    generator->setRobot(&robot_);
    if (not torque_generators_.add(
            name, StorageWrapper<JointForceGenerator>{generator}, force)) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Failed to add generator " + name +
                            ". Use a different name or call "
                            "SafetyController::forceAdd instead to "
                            "replace the existing one."));
    }
}

void SafetyController::addVelocityGenerator(
    const std::string& name, std::shared_ptr<VelocityGenerator> generator,
    bool force) {
    generator->setRobot(&robot_);
    generator->setFrame(robot_.controlPointFrame());
    if (not velocity_generators_.add(
            name,
            StorageWrapper<VelocityGenerator>{
                generator, spatial::Velocity::Zero(robot_.controlPointFrame())},
            force)) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Failed to add generator " + name +
                            ". Use a different name or call "
                            "SafetyController::forceAdd instead to "
                            "replace the existing one."));
    }
}

void SafetyController::addJointVelocityGenerator(
    const std::string& name, std::shared_ptr<JointVelocityGenerator> generator,
    bool force) {
    generator->setRobot(&robot_);
    if (not joint_velocity_generators_.add(
            name, StorageWrapper<JointVelocityGenerator>{generator}, force)) {
        throw std::runtime_error(
            OPEN_PHRI_ERROR("Failed to add generator " + name +
                            ". Use a different name or call "
                            "SafetyController::forceAdd instead to "
                            "replace the existing one."));
    }
}

bool SafetyController::removeConstraint(const std::string& name) {
    return constraints_.remove(name);
}

bool SafetyController::removeForceGenerator(const std::string& name) {
    return force_generators_.remove(name);
}

bool SafetyController::removeJointForceGenerator(const std::string& name) {
    return torque_generators_.remove(name);
}

bool SafetyController::removeVelocityGenerator(const std::string& name) {
    return velocity_generators_.remove(name);
}

bool SafetyController::removeJointVelocityGenerator(const std::string& name) {
    return joint_velocity_generators_.remove(name);
}

std::shared_ptr<Constraint>
SafetyController::getConstraint(const std::string& name) {
    return constraints_.get(name).object;
}

std::shared_ptr<ForceGenerator>
SafetyController::getForceGenerator(const std::string& name) {
    return force_generators_.get(name).object;
}

std::shared_ptr<VelocityGenerator>
SafetyController::getVelocityGenerator(const std::string& name) {
    return velocity_generators_.get(name).object;
}

std::shared_ptr<JointForceGenerator>
SafetyController::getJointForceGenerator(const std::string& name) {
    return torque_generators_.get(name).object;
}

std::shared_ptr<JointVelocityGenerator>
SafetyController::getJointVelocityGenerator(const std::string& name) {
    return joint_velocity_generators_.get(name).object;
}

void SafetyController::removeAll() {
    removeAllVelocityGenerators();
    removeAllJointVelocityGenerators();
    removeAllForceGenerators();
    removeAllJointForceGenerators();
    removeAllConstraints();
}

void SafetyController::removeAllVelocityGenerators() {
    velocity_generators_.removeAll();
}

void SafetyController::removeAllJointVelocityGenerators() {
    joint_velocity_generators_.removeAll();
}

void SafetyController::removeAllForceGenerators() {
    force_generators_.removeAll();
}

void SafetyController::removeAllJointForceGenerators() {
    torque_generators_.removeAll();
}

void SafetyController::removeAllConstraints() {
    constraints_.removeAll();
}

void SafetyController::enableDampedLeastSquares(double lambda) {
    assert(lambda > 0.);
    lambda2_ = lambda * lambda;
    dynamic_dls_ = false;
}

void SafetyController::enableDynamicDampedLeastSquares(
    double lambda_max, double sigma_min_threshold) {
    assert(lambda_max > 0. and sigma_min_threshold > 0.);
    lambda2_ = lambda_max * lambda_max;
    sigma_min_threshold_ = sigma_min_threshold;
    dynamic_dls_ = true;
}

void SafetyController::compute() {
    const auto& jacobian_inverse = robot_.control().jacobianInverse();
    const auto& jacobian = robot_.control().jacobian();
    // const auto& spatial_transformation =
    //     robot_.control().spatial_transformation_matrix;
    const auto& transformation = robot_.control().transformation();
    if (not skip_jacobian_inverse_computation_) {
        robot_.control().jacobian_inverse_ = computeJacobianInverse();
    }

    const auto& force_sum = computeForceSum();
    const auto& torque_sum = computeTorqueSum();
    const auto& velocity_sum = computeVelocitySum();
    const auto& joint_velocity_sum = computeJointVelocitySum();
#if HEAVY_PRINTING
    std::cout << "[phri::phSafetyController::compute]\n";
    std::cout << "\t###########################################################"
                 "########\n";
    std::cout << "\tforce_sum: " << force_sum.transpose() << std::endl;
    std::cout << "\ttorque_sum: " << torque_sum.transpose() << std::endl;
    std::cout << "\tvelocity_sum: " << velocity_sum.transpose() << std::endl;
    std::cout << "\tjoint_velocity_sum: " << joint_velocity_sum.transpose()
              << std::endl;
    // std::cout << "\tspatial_transformation:\n " << spatial_transformation
    //           << std::endl;
    std::cout << "\tjacobian:\n " << jacobian << std::endl;
    std::cout << "\tjacobian_inverse:\n " << jacobian_inverse << std::endl;
    std::cout << "\t***********************************************************"
                 "********\n";
#endif

    // Joint level damping control
    robot_.control().joints().velocity_command_ =
        torque_sum / robot_.control().joints().damping() + joint_velocity_sum;

    // Control point level damping control
    robot_.control().task().velocity_command_ =
        force_sum / robot_.control().task().damping() + velocity_sum;

    // Cumulative effect on the joint velocity of all inputs
    robot_.control().joints().total_velocity_.value() =
        jacobian_inverse *
            (transformation * robot_.control().task().velocityCommand()) +
        robot_.control().joints().velocityCommand();

    // Cumulative effect on the control point velocity of all inputs
    robot_.control().task().total_velocity_ =
        robot_.control().task().velocityCommand() +
        transformation.inverse() *
            spatial::Velocity{jacobian *
                                  robot_.control().joints().velocityCommand(),
                              robot_.controlPointParentFrame()};

    // Cumulative effect on torques of both joint torque and control point
    // force inputs
    robot_.control().joints().total_force_ =
        torque_sum + jacobian.transpose() * force_sum;

    // Cumulative effect on forces of both joint torque and control point
    // force inputs
    robot_.control().task().total_force_ =
        force_sum + jacobian_inverse.transpose() * torque_sum;

    // Compute the velocity scaling factor
    double constraint_value = computeConstraintValue();

    // Scale the joint velocities to comply with the constraints
    robot_.joints().command_.velocity() =
        robot_.control().joints().totalVelocity() * constraint_value;

    // Scale the control point velocities to comply with the constraints
    robot_.task().command_.velocity() =
        robot_.control().task().totalVelocity() * constraint_value;

#if HEAVY_PRINTING
    std::cout << "\tjoint vel cmd: "
              << robot_.control().joints.velocity_command.transpose()
              << std::endl;
    std::cout << "\tcp vel cmd: " << robot_.task().command.twist << std::endl;
    std::cout << "\tjoint vel tot: "
              << robot_.control().joints.total_velocity.transpose()
              << std::endl;
    std::cout << "\tcp vel tot: " << robot_.control().task.total_twist
              << std::endl;
    std::cout << "\tjont trq tot: "
              << robot_.control().joints.total_force.transpose() << std::endl;
    std::cout << "\tcp force tot: " << robot_.control().task.total_wrench
              << std::endl;
    std::cout << "\tjoint vel: " << robot_.joints().command.velocity.transpose()
              << std::endl;
    std::cout << "\tcp vel: " << robot_.task().command.twist << std::endl;
#endif
}

void SafetyController::print() const {
    std::cout << "[phri::SafetyController::print]\n";
    std::cout << "\tForce generators:\n";
    for (const auto& gen : force_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.cref().object)
                  << "): " << gen.second.cref().last_value << "\n";
    }
    std::cout << "\tVelocity generators:\n";
    for (const auto& gen : velocity_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.cref().object)
                  << "): " << gen.second.cref().last_value << "\n";
    }
    std::cout << "\tTorque generators:\n";
    for (const auto& gen : torque_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.cref().object)
                  << "): " << gen.second.cref().last_value.transpose() << "\n";
    }
    std::cout << "\tJoint velocity generators:\n";
    for (const auto& gen : joint_velocity_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.cref().object)
                  << "): " << gen.second.cref().last_value.transpose() << "\n";
    }
    std::cout << "\tConstraints:\n";
    for (const auto& cstr : constraints_) {
        std::cout << "\t\t- " << cstr.first << " ("
                  << getTypeName(*cstr.second.cref().object)
                  << "): " << cstr.second.cref().last_value << "\n";
    }
}

void SafetyController::operator()() {
    compute();
}

SafetyController::storage_const_iterator<Constraint>
SafetyController::constraints_begin() const {
    return constraints_.begin();
}
SafetyController::storage_const_iterator<Constraint>
SafetyController::constraints_end() const {
    return constraints_.end();
}

SafetyController::storage_const_iterator<ForceGenerator>
SafetyController::force_generators_begin() const {
    return force_generators_.begin();
}
SafetyController::storage_const_iterator<ForceGenerator>
SafetyController::force_generators_end() const {
    return force_generators_.end();
}

SafetyController::storage_const_iterator<JointForceGenerator>
SafetyController::torque_generators_begin() const {
    return torque_generators_.begin();
}
SafetyController::storage_const_iterator<JointForceGenerator>
SafetyController::torque_generators_end() const {
    return torque_generators_.end();
}

SafetyController::storage_const_iterator<VelocityGenerator>
SafetyController::velocity_generators_begin() const {
    return velocity_generators_.begin();
}
SafetyController::storage_const_iterator<VelocityGenerator>
SafetyController::velocity_generators_end() const {
    return velocity_generators_.end();
}

SafetyController::storage_const_iterator<JointVelocityGenerator>
SafetyController::joint_velocity_generators_begin() const {
    return joint_velocity_generators_.begin();
}
SafetyController::storage_const_iterator<JointVelocityGenerator>
SafetyController::joint_velocity_generators_end() const {
    return joint_velocity_generators_.end();
}

double SafetyController::computeConstraintValue() {
    double min_value = std::numeric_limits<double>::infinity();

    for (auto& constraint : constraints_) {
        constraint.second.ref().last_value =
            constraint.second.ref().object->compute();
        min_value = std::min(min_value, constraint.second.cref().last_value);
    }

    return robot_.control().scaling_factor_ = std::min(1., min_value);
}

const spatial::Force& SafetyController::computeForceSum() {
    auto& sum = robot_.control().task().force_sum_;
    sum.setZero();

    for (auto& force_generator : force_generators_) {
        force_generator.second.ref().last_value =
            force_generator.second.ref().object->compute();
        sum += force_generator.second.cref().last_value;
    }

    return sum;
}

const vector::dyn::Force& SafetyController::computeTorqueSum() {
    auto& sum = robot_.control().joints().force_sum_;
    sum.setZero();

    for (auto& torque_generator : torque_generators_) {
        torque_generator.second.ref().last_value =
            torque_generator.second.ref().object->compute();
        sum += torque_generator.second.cref().last_value;
    }

    return sum;
}

const spatial::Velocity& SafetyController::computeVelocitySum() {
    auto& sum = robot_.control().task().velocity_sum_;
    sum.setZero();

    for (auto& velocity_generator : velocity_generators_) {
        velocity_generator.second.ref().last_value =
            velocity_generator.second.ref().object->compute();
        sum += velocity_generator.second.cref().last_value;
    }

    return sum;
}

const vector::dyn::Velocity& SafetyController::computeJointVelocitySum() {
    auto& sum = robot_.control().joints().velocity_sum_;
    sum.setZero();

    for (auto& joint_velocity_generator : joint_velocity_generators_) {
        joint_velocity_generator.second.ref().last_value =
            joint_velocity_generator.second.ref().object->compute();
        sum += joint_velocity_generator.second.cref().last_value;
    }

    return sum;
}

const Eigen::MatrixXd& SafetyController::computeJacobianInverse() const {
    auto& jac = robot_.control().jacobian();
    auto& jac_inv = robot_.control().jacobian_inverse_;

    if (jac.rows() == jac.cols()) {
        jac_inv = jac.inverse();
    } else {
        if (lambda2_ > 0.) {
            double lambda2;

            if (dynamic_dls_) {
                Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac);
                Eigen::VectorXd sigmas = svd.singularValues();
                double sigma_min = sigmas.tail<1>()(0);
                if (sigma_min > sigma_min_threshold_) {
                    lambda2 = 0.;
                } else {
                    lambda2 =
                        lambda2_ *
                        (1. - std::pow(sigma_min / sigma_min_threshold_, 2));
                }
            } else {
                lambda2 = lambda2_;
            }

            Eigen::Matrix<double, 6, 6> tmp;
            tmp = jac * jac.transpose();
            tmp += lambda2 * Eigen::Matrix<double, 6, 6>::Identity();
            tmp = tmp.inverse();

            jac_inv = jac.transpose() * tmp;
        } else {
            jac_inv = jac.pseudoInverse();
        }
    }

    return jac_inv;
}
