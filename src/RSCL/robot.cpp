#include <RSCL/robot.h>

using namespace RSCL;

Robot::Robot(const std::string& name,
             size_t joint_count) :
	name_(name),
	joint_count_(joint_count)
{
	joint_damping_matrix_               = std::make_shared<MatrixXd>();
	control_point_damping_matrix_       = std::make_shared<Matrix6d>(Matrix6d::Identity());

	joint_velocity_                     = std::make_shared<Eigen::VectorXd>();
	joint_velocity_sum_                 = std::make_shared<Eigen::VectorXd>();
	joint_torque_sum_                   = std::make_shared<Eigen::VectorXd>();
	joint_velocity_command_             = std::make_shared<Eigen::VectorXd>();
	joint_total_velocity_               = std::make_shared<Eigen::VectorXd>();
	joint_total_torque_                 = std::make_shared<Eigen::VectorXd>();

	joint_current_position_             = std::make_shared<Eigen::VectorXd>();
	joint_target_position_              = std::make_shared<Eigen::VectorXd>();
	joint_external_torque_              = std::make_shared<Eigen::VectorXd>();

	control_point_velocity_             = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_velocity_sum_         = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_force_sum_            = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_velocity_command_     = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_total_velocity_       = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_total_force_          = std::make_shared<Vector6d>(Vector6d::Zero());

	control_point_current_pose_         = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_target_pose_          = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_current_velocity_     = std::make_shared<Vector6d>(Vector6d::Zero());
	control_point_external_force_       = std::make_shared<Vector6d>(Vector6d::Zero());

	scaling_factor_                     = std::make_shared<double>();

	jacobian_                           = std::make_shared<MatrixXd>();
	jacobian_inverse_                   = std::make_shared<MatrixXd>();
	transformation_matrix_              = std::make_shared<Matrix4d>(Matrix4d::Identity());
	spatial_transformation_matrix_      = std::make_shared<Matrix6d>(Matrix6d::Identity());

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

	joint_damping_matrix_->resize(joint_count, joint_count);
	joint_damping_matrix_->setIdentity();
}

const std::string& Robot::name() const {
	return name_;
}

size_t Robot::jointCount() const {
	return joint_count_;
}

MatrixXdPtr Robot::jointDampingMatrix() const {
	return joint_damping_matrix_;
}

Matrix6dPtr Robot::controlPointDampingMatrix() const {
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


Vector6dConstPtr Robot::controlPointVelocity() const {
	return control_point_velocity_;
}

Vector6dConstPtr Robot::controlPointVelocitySum() const {
	return control_point_velocity_sum_;
}

Vector6dConstPtr Robot::controlPointForceSum() const {
	return control_point_force_sum_;
}

Vector6dConstPtr Robot::controlPointVelocityCommand() const {
	return control_point_velocity_command_;
}

Vector6dConstPtr Robot::controlPointTotalVelocity() const {
	return control_point_total_velocity_;
}

Vector6dConstPtr Robot::controlPointTotalForce() const {
	return control_point_total_force_;
}

Vector6dPtr Robot::controlPointCurrentPose() const {
	return control_point_current_pose_;
}

Vector6dPtr Robot::controlPointTargetPose() const {
	return control_point_target_pose_;
}

Vector6dPtr Robot::controlPointCurrentVelocity() const {
	return control_point_current_velocity_;
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
