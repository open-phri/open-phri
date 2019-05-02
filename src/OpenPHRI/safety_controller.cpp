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
#include <OpenPHRI/torque_generators/torque_generator.h>
#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/utilities/demangle.h>

#include <Eigen/SVD>
#include <yaml-cpp/yaml.h>

#include <limits>
#include <iostream>

using namespace phri;

#define HEAVY_PRINTING 1

SafetyController::SafetyController(Robot& robot)
    : robot_(robot), skip_jacobian_inverse_computation_(false) {
    addConstraint("default constraint", std::make_shared<DefaultConstraint>());
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
                                  "TorqueGenerator");
    velocity_generators_.setVerbose(on, "phri::SafetyController",
                                    "VelocityGenerator");
    joint_velocity_generators_.setVerbose(on, "phri::SafetyController",
                                          "JointVelocityGenerator");
}

void SafetyController::skipJacobianInverseComputation(bool on) {
    skip_jacobian_inverse_computation_ = on;
}

bool SafetyController::addConstraint(const std::string& name,
                                     ConstraintPtr constraint, bool force) {
    constraint->setRobot(&robot_);
    return constraints_.add(name, {constraint}, force);
}

bool SafetyController::addForceGenerator(const std::string& name,
                                         ForceGeneratorPtr generator,
                                         bool force) {
    generator->setRobot(&robot_);
    return force_generators_.add(name, {generator}, force);
}

bool SafetyController::addTorqueGenerator(const std::string& name,
                                          TorqueGeneratorPtr generator,
                                          bool force) {
    generator->setRobot(&robot_);
    return torque_generators_.add(name, {generator}, force);
}

bool SafetyController::addVelocityGenerator(const std::string& name,
                                            VelocityGeneratorPtr generator,
                                            bool force) {
    generator->setRobot(&robot_);
    return velocity_generators_.add(name, {generator}, force);
}

bool SafetyController::addJointVelocityGenerator(
    const std::string& name, JointVelocityGeneratorPtr generator, bool force) {
    generator->setRobot(&robot_);
    return joint_velocity_generators_.add(name, {generator}, force);
}

bool SafetyController::removeConstraint(const std::string& name) {
    return constraints_.remove(name);
}

bool SafetyController::removeForceGenerator(const std::string& name) {
    return force_generators_.remove(name);
}

bool SafetyController::removeTorqueGenerator(const std::string& name) {
    return torque_generators_.remove(name);
}

bool SafetyController::removeVelocityGenerator(const std::string& name) {
    return velocity_generators_.remove(name);
}

bool SafetyController::removeJointVelocityGenerator(const std::string& name) {
    return joint_velocity_generators_.remove(name);
}

ConstraintPtr SafetyController::getConstraint(const std::string& name) {
    return constraints_.get(name).object;
}

ForceGeneratorPtr SafetyController::getForceGenerator(const std::string& name) {
    return force_generators_.get(name).object;
}

TorqueGeneratorPtr
SafetyController::getTorqueGenerator(const std::string& name) {
    return torque_generators_.get(name).object;
}

JointVelocityGeneratorPtr
SafetyController::getJointVelocityGenerator(const std::string& name) {
    return joint_velocity_generators_.get(name).object;
}

void SafetyController::removeAll() {
    removeAllVelocityInputs();
    removeAllJointVelocityInputs();
    removeAllForceInputs();
    removeAllTorqueInputs();
    removeAllConstraints();
}

void SafetyController::removeAllVelocityInputs() {
    velocity_generators_.removeAll();
}

void SafetyController::removeAllJointVelocityInputs() {
    joint_velocity_generators_.removeAll();
}

void SafetyController::removeAllForceInputs() {
    force_generators_.removeAll();
}

void SafetyController::removeAllTorqueInputs() {
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
    const auto& jacobian_inverse = robot_.control.jacobian_inverse;
    const auto& jacobian = robot_.control.jacobian;
    const auto& spatial_transformation =
        robot_.control.spatial_transformation_matrix;
    if (not skip_jacobian_inverse_computation_) {
        robot_.control.jacobian_inverse = computeJacobianInverse();
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
    std::cout << "\tspatial_transformation:\n " << spatial_transformation
              << std::endl;
    std::cout << "\tjacobian:\n " << jacobian << std::endl;
    std::cout << "\tjacobian_inverse:\n " << jacobian_inverse << std::endl;
    std::cout << "\t***********************************************************"
                 "********\n";
#endif

    // Joint level damping control
    robot_.control.joints.velocity_command =
        torque_sum.cwiseQuotient(robot_.control.joints.damping) +
        joint_velocity_sum;

    // Control point level damping control
    robot_.control.task.twist_command =
        force_sum.cwiseQuotient(robot_.control.task.damping) +
        static_cast<Vector6d>(velocity_sum);

    // Cumulative effect on the joint velocity of all inputs
    robot_.control.joints.total_velocity =
        jacobian_inverse * spatial_transformation *
            static_cast<Vector6d>(robot_.control.task.twist_command) +
        robot_.control.joints.velocity_command;

    // Cumulative effect on the control point velocity of all inputs
    robot_.control.task.total_twist =
        static_cast<Vector6d>(robot_.control.task.twist_command) +
        spatial_transformation.transpose() * jacobian *
            robot_.control.joints.velocity_command;

    // Cumulative effect on torques of both joint torque and control point force
    // inputs
    robot_.control.joints.total_force =
        torque_sum + jacobian.transpose() * force_sum;

    // Cumulative effect on forces of both joint torque and control point force
    // inputs
    robot_.control.task.total_wrench =
        force_sum + jacobian_inverse.transpose() * torque_sum;

    // Compute the velocity scaling factor
    double constraint_value = computeConstraintValue();

    // Scale the joint velocities to comply with the constraints
    robot_.joints.command.velocity =
        constraint_value * robot_.control.joints.total_velocity;

    // Scale the control point velocities to comply with the constraints
    robot_.task.command.twist =
        constraint_value *
        static_cast<Vector6d>(robot_.control.task.total_twist);

#if HEAVY_PRINTING
    std::cout << "\tjoint vel cmd: "
              << robot_.control.joints.velocity_command.transpose()
              << std::endl;
    std::cout << "\tcp vel cmd: " << robot_.task.command.twist << std::endl;
    std::cout << "\tjoint vel tot: "
              << robot_.control.joints.total_velocity.transpose() << std::endl;
    std::cout << "\tcp vel tot: " << robot_.control.task.total_twist
              << std::endl;
    std::cout << "\tjont trq tot: "
              << robot_.control.joints.total_force.transpose() << std::endl;
    std::cout << "\tcp force tot: " << robot_.control.task.total_wrench
              << std::endl;
    std::cout << "\tjoint vel: " << robot_.joints.command.velocity.transpose()
              << std::endl;
    std::cout << "\tcp vel: " << robot_.task.command.twist << std::endl;
#endif
}

void SafetyController::print() const {
    std::cout << "[phri::SafetyController::print]\n";
    std::cout << "\tForce generators:\n";
    for (const auto& gen : force_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.object)
                  << "): " << gen.second.last_value.transpose() << "\n";
    }
    std::cout << "\tVelocity generators:\n";
    for (const auto& gen : velocity_generators_) {
        std::cout
            << "\t\t- " << gen.first << " (" << getTypeName(*gen.second.object)
            << "): "
            << static_cast<const Vector6d&>(gen.second.last_value).transpose()
            << "\n";
    }
    std::cout << "\tTorque generators:\n";
    for (const auto& gen : torque_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.object)
                  << "): " << gen.second.last_value.transpose() << "\n";
    }
    std::cout << "\tJoint velocity generators:\n";
    for (const auto& gen : joint_velocity_generators_) {
        std::cout << "\t\t- " << gen.first << " ("
                  << getTypeName(*gen.second.object)
                  << "): " << gen.second.last_value.transpose() << "\n";
    }
    std::cout << "\tConstraints:\n";
    for (const auto& cstr : constraints_) {
        std::cout << "\t\t- " << cstr.first << " ("
                  << getTypeName(*cstr.second.object)
                  << "): " << cstr.second.last_value << "\n";
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

SafetyController::storage_const_iterator<TorqueGenerator>
SafetyController::torque_generators_begin() const {
    return torque_generators_.begin();
}
SafetyController::storage_const_iterator<TorqueGenerator>
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
        constraint.second.last_value = constraint.second.object->compute();
        min_value = std::min(min_value, constraint.second.last_value);
    }

    return robot_.control.scaling_factor = std::min(1., min_value);
}

const Vector6d& SafetyController::computeForceSum() {
    auto& sum = static_cast<Vector6d&>(robot_.control.task.wrench_sum);
    sum.setZero();

    for (auto& force_generator : force_generators_) {
        force_generator.second.last_value =
            force_generator.second.object->compute();
        sum += force_generator.second.last_value;
    }

    return sum;
}

const VectorXd& SafetyController::computeTorqueSum() {
    auto& sum = robot_.control.joints.force_sum;
    sum.setZero();

    for (auto& torque_generator : torque_generators_) {
        torque_generator.second.last_value =
            torque_generator.second.object->compute();
        sum += torque_generator.second.last_value;
    }

    return sum;
}

const Vector6d& SafetyController::computeVelocitySum() {
    auto& sum = static_cast<Vector6d&>(robot_.control.task.twist_sum);
    sum.setZero();

    for (auto& velocity_generator : velocity_generators_) {
        velocity_generator.second.last_value =
            velocity_generator.second.object->compute();
        sum +=
            static_cast<const Vector6d&>(velocity_generator.second.last_value);
    }

    return sum;
}

const VectorXd& SafetyController::computeJointVelocitySum() {
    auto& sum = robot_.control.joints.velocity_sum;
    sum.setZero();

    for (auto& joint_velocity_generator : joint_velocity_generators_) {
        joint_velocity_generator.second.last_value =
            joint_velocity_generator.second.object->compute();
        sum += joint_velocity_generator.second.last_value;
    }

    return sum;
}

const MatrixXd& SafetyController::computeJacobianInverse() const {
    auto& jac = robot_.control.jacobian;
    auto& jac_inv = robot_.control.jacobian_inverse;

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
