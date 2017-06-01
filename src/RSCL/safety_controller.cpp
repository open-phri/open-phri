#include <RSCL/safety_controller.h>

#include <RSCL/constraints/default_constraint.h>
#include <RSCL/force_generators/force_generator.h>
#include <RSCL/torque_generators/torque_generator.h>
#include <RSCL/velocity_generators/velocity_generator.h>
#include <RSCL/joint_velocity_generators/joint_velocity_generator.h>

#include <limits>
#include <iostream>

using namespace RSCL;

SafetyController::SafetyController() {
	addConstraint("default", std::make_shared<DefaultConstraint>());
}

SafetyController::SafetyController(
	RobotPtr robot) :
	SafetyController()
{
	robot_ = robot;
}

void SafetyController::setVerbose(bool on) {
	constraints_.setVerbose                 (on, "RSCL::SafetyController", "Constraint");
	force_generators_.setVerbose            (on, "RSCL::SafetyController", "ForceGenerator");
	torque_generators_.setVerbose           (on, "RSCL::SafetyController", "TorqueGenerator");
	velocity_generators_.setVerbose         (on, "RSCL::SafetyController", "VelocityGenerator");
	joint_velocity_generators_.setVerbose   (on, "RSCL::SafetyController", "JointVelocityGenerator");
}

bool SafetyController::addConstraint(const std::string& name, ConstraintPtr constraint, bool force) {
	constraint->setRobot(robot_);

	return constraints_.add(name, constraint, force);
}

bool SafetyController::addForceGenerator(const std::string& name, ForceGeneratorPtr generator, bool force) {
	return force_generators_.add(name, generator, force);
}

bool SafetyController::addTorqueGenerator(const std::string& name, TorqueGeneratorPtr generator, bool force) {
	return torque_generators_.add(name, generator, force);
}

bool SafetyController::addVelocityGenerator(const std::string& name, VelocityGeneratorPtr generator, bool force) {
	return velocity_generators_.add(name, generator, force);
}

bool SafetyController::addJointVelocityGenerator(const std::string& name, JointVelocityGeneratorPtr generator, bool force) {
	return joint_velocity_generators_.add(name, generator, force);
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
	return constraints_.get(name);
}

ForceGeneratorPtr SafetyController::getForceGenerator(const std::string& name) {
	return force_generators_.get(name);
}

TorqueGeneratorPtr SafetyController::getTorqueGenerator(const std::string& name) {
	return torque_generators_.get(name);
}

JointVelocityGeneratorPtr SafetyController::getJointVelocityGenerator(const std::string& name) {
	return joint_velocity_generators_.get(name);
}


void SafetyController::compute() {
	// std::cout << "###################################################################\n";
	const auto& force_sum = computeForceSum();
	const auto& torque_sum = computeTorqueSum();
	const auto& velocity_sum = computeVelocitySum();
	const auto& joint_velocity_sum = computeJointVelocitySum();
	const auto& spatial_transformation = computeSpatialTransformation();
	const auto& jacobian_inverse = computeJacobianInverse();
	const auto& jacobian = *robot_->jacobian();

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
	*robot_->joint_velocity_command_ = robot_->joint_damping_matrix_->inverse() * torque_sum + joint_velocity_sum;
	// std::cout << "joint vel cmd: " << robot_->joint_velocity_command_->transpose() << std::endl;

	// Control point level damping control
	*robot_->control_point_velocity_command_ = robot_->control_point_damping_matrix_->inverse() * force_sum + velocity_sum;
	// std::cout << "cp vel cmd: " << robot_->control_point_velocity_command_->transpose() << std::endl;

	// Cumulative effect on the joint velocity of all inputs
	*robot_->joint_total_velocity_ = jacobian_inverse * spatial_transformation * *robot_->control_point_velocity_command_ + *robot_->joint_velocity_command_;
	// std::cout << "joint vel tot: " << robot_->joint_total_velocity_->transpose() << std::endl;

	// Cumulative effect on the control point velocity of all inputs
	*robot_->control_point_total_velocity_ = *robot_->control_point_velocity_command_ + spatial_transformation.transpose() * jacobian * *robot_->joint_velocity_command_;
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
	// std::cout << "joint vel: " << robot_->joint_velocity_->transpose() << std::endl;

	// Scale the control point velocities to comply with the constraints
	*robot_->control_point_velocity_ = constraint_value * *robot_->control_point_total_velocity_;
	// std::cout << "cp vel: " << robot_->control_point_velocity_->transpose() << std::endl;
}

double SafetyController::computeConstraintValue() const {
	double min_value = std::numeric_limits<double>::infinity();

	for(const auto& constraint : constraints_) {
		min_value = std::min(min_value, constraint.second->compute());
	}

	return *robot_->scaling_factor_ = std::min(1., min_value);
}

const Vector6d& SafetyController::computeForceSum() const {
	auto& sum = *robot_->control_point_force_sum_;
	sum.setZero();

	for(const auto& force_generator : force_generators_) {
		sum += force_generator.second->compute();
	}

	return sum;
}

const VectorXd&  SafetyController::computeTorqueSum() const {
	auto& sum = *robot_->joint_torque_sum_;
	sum.setZero();

	for(const auto& torque_generator : torque_generators_) {
		sum += torque_generator.second->compute();
	}

	return sum;
}

const Vector6d&  SafetyController::computeVelocitySum() const {
	auto& sum = *robot_->control_point_velocity_sum_;
	sum.setZero();

	for(const auto& velocity_generator : velocity_generators_) {
		sum += velocity_generator.second->compute();
	}

	return sum;
}

const VectorXd& SafetyController::computeJointVelocitySum() const {
	auto& sum = *robot_->joint_velocity_sum_;
	sum.setZero();

	for(const auto& joint_velocity_generator : joint_velocity_generators_) {
		sum += joint_velocity_generator.second->compute();
	}

	return sum;
}

const Matrix6d& SafetyController::computeSpatialTransformation() const {
	auto& mat = *robot_->spatial_transformation_matrix_;
	const auto& rot_mat = robot_->transformation_matrix_->block<3,3>(0,0);
	mat.block<3,3>(3,0).setZero();
	mat.block<3,3>(0,0) = rot_mat;
	mat.block<3,3>(0,3).setZero();
	mat.block<3,3>(3,3) = rot_mat;

	return mat;
}

const MatrixXd& SafetyController::computeJacobianInverse() const {
	auto& jac = *robot_->jacobian_;
	auto& jac_inv = *robot_->jacobian_inverse_;

	if(jac.rows() == jac.cols()) {
		jac_inv = jac.inverse();
	}
	else {
		// Compute the pseudo inverse in the non square case based on SVD decomposition
		Eigen::JacobiSVD<MatrixXd> svd(jac, Eigen::ComputeThinU | Eigen::ComputeThinV);
		double tolerance = std::numeric_limits<double>::epsilon() * std::max(jac.cols(), jac.rows()) * svd.singularValues().array().abs()(0);
		jac_inv = svd.matrixV() * (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
	}

	return jac_inv;
}
