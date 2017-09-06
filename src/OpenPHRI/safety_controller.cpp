/*      File: safety_controller.cpp
*       This file is part of the program open-phri
*       Program description : OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
*       Copyright (C) 2017 -  Benjamin Navarro (LIRMM). All Right reserved.
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
*       You should have received a copy of the GNU Lesser General Public License version 3 and the
*       General Public License version 3 along with this program.
*       If not, see <http://www.gnu.org/licenses/>.
*/

#include <OpenPHRI/safety_controller.h>

#include <OpenPHRI/constraints/default_constraint.h>
#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/torque_generators/torque_generator.h>
#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/utilities/demangle.h>

#include <Eigen/SVD>

#include <limits>
#include <iostream>

using namespace phri;

SafetyController::SafetyController(
	RobotPtr robot) :
	skip_jacobian_inverse_computation_(false)
{
	addConstraint("default constraint", std::make_shared<DefaultConstraint>());
	robot_ = robot;
	null_space_velocity_ = std::make_shared<VectorXd>(robot_->jointCount());
	null_space_velocity_->setZero();
	dynamic_dls_ = false;
	lambda2_ = -1.;
	sigma_min_threshold_ = -1.;
}

void SafetyController::setVerbose(bool on) {
	constraints_.setVerbose                 (on, "phri::SafetyController", "Constraint");
	force_generators_.setVerbose            (on, "phri::SafetyController", "ForceGenerator");
	torque_generators_.setVerbose           (on, "phri::SafetyController", "TorqueGenerator");
	velocity_generators_.setVerbose         (on, "phri::SafetyController", "VelocityGenerator");
	joint_velocity_generators_.setVerbose   (on, "phri::SafetyController", "JointVelocityGenerator");
}

void SafetyController::skipJacobianInverseComputation(bool on) {
	skip_jacobian_inverse_computation_ = on;
}

bool SafetyController::addConstraint(const std::string& name, ConstraintPtr constraint, bool force) {
	constraint->setRobot(robot_);
	return constraints_.add(name, {constraint}, force);
}

bool SafetyController::addForceGenerator(const std::string& name, ForceGeneratorPtr generator, bool force) {
	generator->robot_ = robot_;
	return force_generators_.add(name, {generator}, force);
}

bool SafetyController::addTorqueGenerator(const std::string& name, TorqueGeneratorPtr generator, bool force) {
	generator->robot_ = robot_;
	return torque_generators_.add(name, {generator}, force);
}

bool SafetyController::addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator, bool force) {
	generator->robot_ = robot_;
	return velocity_generators_.add(name, {generator}, force);
}

bool SafetyController::addJointVelocityGenerator(const std::string& name, JointVelocityGeneratorPtr generator, bool force) {
	generator->robot_ = robot_;
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

TorqueGeneratorPtr SafetyController::getTorqueGenerator(const std::string& name) {
	return torque_generators_.get(name).object;
}

JointVelocityGeneratorPtr SafetyController::getJointVelocityGenerator(const std::string& name) {
	return joint_velocity_generators_.get(name).object;
}

std::shared_ptr<VectorXd> SafetyController::getNullSpaceVelocityVector() const {
	return null_space_velocity_;
}

void SafetyController::enableDampedLeastSquares(double lambda) {
	assert(lambda > 0.);
	lambda2_ = lambda*lambda;
	dynamic_dls_ = false;
}

void SafetyController::enableDynamicDampedLeastSquares(double lambda_max, double sigma_min_threshold) {
	assert(lambda_max > 0. and sigma_min_threshold > 0.);
	lambda2_ = lambda_max*lambda_max;
	sigma_min_threshold_ = sigma_min_threshold;
	dynamic_dls_ = true;
}


void SafetyController::compute() {
	// std::cout << "###################################################################\n";
	const auto& force_sum = computeForceSum();
	const auto& torque_sum = computeTorqueSum();
	const auto& velocity_sum = computeVelocitySum();
	const auto& joint_velocity_sum = computeJointVelocitySum();
	const auto& jacobian_inverse = *robot_->jacobianInverse();
	const auto& jacobian = *robot_->jacobian();
	const auto& spatial_transformation = *robot_->spatialTransformationMatrix();

	if(not skip_jacobian_inverse_computation_) {
		*robot_->jacobianInverse() = computeJacobianInverse();
	}

	// std::cout << "force_sum: " << force_sum.transpose() << std::endl;
	// std::cout << "torque_sum: " << torque_sum.transpose() << std::endl;
	// std::cout << "velocity_sum: " << velocity_sum.transpose() << std::endl;
	// std::cout << "joint_velocity_sum: " << joint_velocity_sum.transpose() << std::endl;
	// std::cout << "spatial_transformation:\n " << spatial_transformation << std::endl;
	// std::cout << "jacobian:\n " << jacobian << std::endl;
	// std::cout << "jacobian_inverse:\n " << jacobian_inverse << std::endl;
	//
	// std::cout << "*******************************************************************\n";

	// Joint level damping control
	*robot_->joint_velocity_command_ = torque_sum.cwiseQuotient(*robot_->joint_damping_matrix_) + joint_velocity_sum;
	// std::cout << "joint vel cmd: " << robot_->joint_velocity_command_->transpose() << std::endl;

	// Control point level damping control
	*robot_->control_point_velocity_command_ = force_sum.cwiseQuotient(*robot_->control_point_damping_matrix_) + static_cast<Vector6d>(velocity_sum);
	// std::cout << "cp vel cmd: " << robot_->control_point_velocity_command_->transpose() << std::endl;

	// Cumulative effect on the joint velocity of all inputs
	*robot_->joint_total_velocity_ = jacobian_inverse * spatial_transformation * static_cast<Vector6d>(*robot_->control_point_velocity_command_) + *robot_->joint_velocity_command_;
	// std::cout << "joint vel tot: " << robot_->joint_total_velocity_->transpose() << std::endl;

	// Cumulative effect on the control point velocity of all inputs
	*robot_->control_point_total_velocity_ = static_cast<Vector6d>(*robot_->control_point_velocity_command_) + spatial_transformation.transpose() * jacobian * *robot_->joint_velocity_command_;
	// std::cout << "cp vel tot: " << robot_->control_point_total_velocity_->transpose() << std::endl;

	// Cumulative effect on torques of both joint torque and control point force inputs
	*robot_->joint_total_torque_ = torque_sum + jacobian.transpose() * force_sum;
	// std::cout << "jont trq tot: " << robot_->joint_total_torque_->transpose() << std::endl;

	// Cumulative effect on forces of both joint torque and control point force inputs
	*robot_->control_point_total_force_ = force_sum + jacobian_inverse.transpose() * torque_sum;
	// std::cout << "cp force tot: " << robot_->control_point_total_force_->transpose() << std::endl;

	// Compute the velocity scaling factor
	double constraint_value = computeConstraintValue();

	// Scale the joint velocities to comply with the constraints
	*robot_->joint_velocity_ = constraint_value * *robot_->joint_total_velocity_;
	// TODO move this to a joint velocity generator so that it is accounted by the constraints
	if(null_space_velocity_.use_count() > 1) {
		phri::MatrixXd eye(robot_->jointCount(), robot_->jointCount());
		eye.setIdentity();
		*robot_->joint_velocity_ += (eye - *robot_->jacobianInverse() * *robot_->jacobian()) * *null_space_velocity_;;
	}
	// std::cout << "joint vel: " << robot_->joint_velocity_->transpose() << std::endl;

	// Scale the control point velocities to comply with the constraints
	*robot_->control_point_velocity_ = constraint_value * static_cast<Vector6d>(*robot_->control_point_total_velocity_);
	// std::cout << "cp vel: " << robot_->control_point_velocity_->transpose() << std::endl;
}

void SafetyController::print() const {
	std::cout << "Force generators:\n";
	for (const auto& gen: force_generators_) {
		std::cout << "\t- " << gen.first << " (" << getTypeName(*gen.second.object) << "): " << gen.second.last_value.transpose() << "\n";
	}
	std::cout << "Velocity generators:\n";
	for (const auto& gen: velocity_generators_) {
		std::cout << "\t- " << gen.first << " (" << getTypeName(*gen.second.object) << "): " << static_cast<const Vector6d&>(gen.second.last_value).transpose() << "\n";
	}
	std::cout << "Torque generators:\n";
	for (const auto& gen: torque_generators_) {
		std::cout << "\t- " << gen.first << " (" << getTypeName(*gen.second.object) << "): " << gen.second.last_value.transpose() << "\n";
	}
	std::cout << "Joint velocity generators:\n";
	for (const auto& gen: joint_velocity_generators_) {
		std::cout << "\t- " << gen.first << " (" << getTypeName(*gen.second.object) << "): " << gen.second.last_value.transpose() << "\n";
	}
	std::cout << "Constraints:\n";
	for (const auto& cstr: constraints_) {
		std::cout << "\t- " << cstr.first << " (" << getTypeName(*cstr.second.object) << "): " << cstr.second.last_value << "\n";
	}
}

void SafetyController::operator()() {
	compute();
}

SafetyController::storage_const_iterator<Constraint> SafetyController::constraints_begin() const {
	return constraints_.begin();
}
SafetyController::storage_const_iterator<Constraint> SafetyController::constraints_end() const {
	return constraints_.end();
}

SafetyController::storage_const_iterator<ForceGenerator> SafetyController::force_generators_begin() const {
	return force_generators_.begin();
}
SafetyController::storage_const_iterator<ForceGenerator> SafetyController::force_generators_end() const {
	return force_generators_.end();
}

SafetyController::storage_const_iterator<TorqueGenerator> SafetyController::torque_generators_begin() const {
	return torque_generators_.begin();
}
SafetyController::storage_const_iterator<TorqueGenerator> SafetyController::torque_generators_end() const {
	return torque_generators_.end();
}

SafetyController::storage_const_iterator<VelocityGenerator> SafetyController::velocity_generators_begin() const {
	return velocity_generators_.begin();
}
SafetyController::storage_const_iterator<VelocityGenerator> SafetyController::velocity_generators_end() const {
	return velocity_generators_.end();
}

SafetyController::storage_const_iterator<JointVelocityGenerator> SafetyController::joint_velocity_generators_begin() const {
	return joint_velocity_generators_.begin();
}
SafetyController::storage_const_iterator<JointVelocityGenerator> SafetyController::joint_velocity_generators_end() const {
	return joint_velocity_generators_.end();
}

double SafetyController::computeConstraintValue() {
	double min_value = std::numeric_limits<double>::infinity();

	for(auto& constraint : constraints_) {
		constraint.second.last_value = constraint.second.object->compute();
		min_value = std::min(min_value, constraint.second.last_value);
	}

	return *robot_->scaling_factor_ = std::min(1., min_value);
}

const Vector6d& SafetyController::computeForceSum() {
	auto& sum = *robot_->control_point_force_sum_;
	sum.setZero();

	for(auto& force_generator : force_generators_) {
		force_generator.second.last_value = force_generator.second.object->compute();
		sum += force_generator.second.last_value;
	}

	return sum;
}

const VectorXd&  SafetyController::computeTorqueSum() {
	auto& sum = *robot_->joint_torque_sum_;
	sum.setZero();

	for(auto& torque_generator : torque_generators_) {
		torque_generator.second.last_value = torque_generator.second.object->compute();
		sum += torque_generator.second.last_value;
	}

	return sum;
}

const Vector6d&  SafetyController::computeVelocitySum() {
	Vector6d& sum = *robot_->control_point_velocity_sum_;
	sum.setZero();

	for(auto& velocity_generator : velocity_generators_) {
		velocity_generator.second.last_value = velocity_generator.second.object->compute();
		sum += static_cast<const Vector6d&>(velocity_generator.second.last_value);
	}

	return sum;
}

const VectorXd& SafetyController::computeJointVelocitySum() {
	auto& sum = *robot_->joint_velocity_sum_;
	sum.setZero();

	for(auto& joint_velocity_generator : joint_velocity_generators_) {
		joint_velocity_generator.second.last_value = joint_velocity_generator.second.object->compute();
		sum += joint_velocity_generator.second.last_value;
	}

	return sum;
}

const MatrixXd& SafetyController::computeJacobianInverse() const {
	auto& jac = *robot_->jacobian_;
	auto& jac_inv = *robot_->jacobian_inverse_;

	if(jac.rows() == jac.cols()) {
		jac_inv = jac.inverse();
	}
	else {
		if(lambda2_ > 0.) {
			double lambda2;

			if(dynamic_dls_ > 0.) {
				Eigen::JacobiSVD<Eigen::MatrixXd> svd(jac);
				double sigma_min = svd.singularValues()(5);
				if(sigma_min > sigma_min_threshold_) {
					lambda2 = 0.;
				}
				else {
					lambda2 = lambda2_ * (1. - pow(sigma_min/sigma_min_threshold_, 2.));
				}
			}
			else {
				lambda2 = lambda2_;
			}

			Eigen::Matrix<double, 6, 6> tmp;
			tmp = jac*jac.transpose();
			tmp += lambda2*Eigen::Matrix<double, 6, 6>::Identity();
			tmp = tmp.inverse();

			jac_inv = jac.transpose() * tmp;
		}
		else {
			// Compute the pseudo inverse in the non square case based on SVD decomposition
			Eigen::JacobiSVD<MatrixXd> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
			double tolerance = std::numeric_limits<double>::epsilon() * std::max(jac.cols(), jac.rows()) * svd.singularValues().array().abs()(0);
			jac_inv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
		}
	}

	return jac_inv;
}
