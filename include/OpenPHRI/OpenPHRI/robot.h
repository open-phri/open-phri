/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of OpenPHRI <https://gite.lirmm.fr/navarro/OpenPHRI>.
 *
 *  OpenPHRI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenPHRI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with OpenPHRI.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file robot.h
 * @author Benjamin Navarro
 * @brief Definition of the Robot class
 * @date May 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>

namespace OpenPHRI {

class Robot {
public:
	Robot(const std::string& name,
	      size_t joint_count);
	~Robot() = default;

	/**
	 * @brief The name given to the robot.
	 * @return A const ref to the name.
	 */
	const std::string& name() const;

	/**
	 * @brief Number of joints of the robot.
	 * @return The number of joints.
	 */
	size_t jointCount() const;

	/**
	 * @brief Damping matrix used by the joint-level damping control.
	 * @return A shared pointer to the matrix.
	 */
	VectorXdPtr jointDampingMatrix() const;

	/**
	 * @brief Damping matrix used by the control point-level damping control.
	 * @return A shared pointer to the matrix.
	 */
	Vector6dPtr controlPointDampingMatrix() const;


	/**
	 * @brief Joint velocity outputed by the controller.
	 * @return A shared pointer to the velocity.
	 */
	VectorXdConstPtr jointVelocity() const;

	/**
	 * @brief Sum of all joint velocity inputs.
	 * @return A shared pointer to the velocity.
	 */
	VectorXdConstPtr jointVelocitySum() const;

	/**
	 * @brief Sum of all joint torque inputs.
	 * @return A shared pointer to the torque.
	 */
	VectorXdConstPtr jointTorqueSum() const;

	/**
	 * @brief Output of the joint-level admittance controller
	 * @return A shared pointer to the velocity.
	 */
	VectorXdConstPtr jointVelocityCommand() const;

	/**
	 * @brief Cumulative effect on the joint velocity of all inputs. Same as jointVelocity but without the scaling factor.
	 * @return A shared pointer to the velocity.
	 */
	VectorXdConstPtr jointTotalVelocity() const;

	/**
	 * @brief Cumulative effect of both joint torque and control point force inputs.
	 * @return A shared pointer to the torque.
	 */
	VectorXdConstPtr jointTotalTorque() const;

	/**
	 * @brief Current joint position.
	 * @return A shared pointer to the position.
	 */
	VectorXdPtr jointCurrentPosition() const;

	/**
	 * @brief Current target joint position.
	 * @return A shared pointer to the position.
	 */
	VectorXdPtr jointTargetPosition() const;

	/**
	 * @brief Current joint external torque, read from the robot.
	 * @return A shared pointer to the torques.
	 */
	VectorXdPtr jointExternalTorque() const;

	/**
	 * @brief Control point velocity outputed by the controller.
	 * @return A shared pointer to the velocity.
	 */
	Vector6dConstPtr controlPointVelocity() const;

	/**
	 * @brief Sum of all control point velocity inputs.
	 * @return A shared pointer to the velocity.
	 */
	Vector6dConstPtr controlPointVelocitySum() const;

	/**
	 * @brief Sum of all control point force inputs.
	 * @return A shared pointer to the force.
	 */
	Vector6dConstPtr controlPointForceSum() const;

	/**
	 * @brief Output of the control point-level admittance controller
	 * @return A shared pointer to the velocity.
	 */
	Vector6dConstPtr controlPointVelocityCommand() const;

	/**
	 * @brief Cumulative effect on the control point velocity of all inputs. Same as controlPointVelocity but without the scaling factor.
	 * @return A shared pointer to the velocity.
	 */
	Vector6dConstPtr controlPointTotalVelocity() const;

	/**
	 * @brief Cumulative effect of both control point force and joint torque inputs.
	 * @return A shared pointer to the force.
	 */
	Vector6dConstPtr controlPointTotalForce() const;

	/**
	 * @brief Current control point pose.
	 * @return A shared pointer to the pose.
	 */
	Vector6dPtr controlPointCurrentPose() const;

	/**
	 * @brief Current target control point pose.
	 * @return A shared pointer to the pose.
	 */
	Vector6dPtr controlPointTargetPose() const;

	/**
	 * @brief Current control point veloicty.
	 * @return A shared pointer to the velocity.
	 */
	Vector6dPtr controlPointCurrentVelocity() const;

	/**
	 * @brief Current control point acceleration.
	 * @return A shared pointer to the acceleration.
	 */
	Vector6dPtr controlPointCurrentAcceleration() const;

	/**
	 * @brief Current control point pose external, read from the robot.
	 * @return A shared pointer to the force.
	 */
	Vector6dPtr controlPointExternalForce() const;

	/**
	 * @brief Scaling factor (alpha) used to comply with the constraints.
	 * @return A shared pointer to the value.
	 */
	doubleConstPtr scalingFactor() const;

	/**
	 * @brief The Jacobian associated with the control point.
	 * @return A shared pointer to the matrix.
	 */
	MatrixXdPtr jacobian() const;

	/**
	 * @brief The Jacobian (pseudo-)inverse associated with the control point.
	 * @return A shared pointer to the matrix.
	 */
	MatrixXdPtr jacobianInverse() const;

	/**
	 * @brief The transformation matrix (control point -> base) associated with the control point.
	 * @return A shared pointer to the matrix.
	 */
	Matrix4dPtr transformationMatrix() const;

	/**
	 * @brief The spatial transformation matrix (control point -> base) associated with the control point.
	 * @return A shared pointer to the matrix.
	 */
	Matrix6dPtr spatialTransformationMatrix() const;

private:
	friend class SafetyController;

	std::string name_;
	size_t joint_count_;
	VectorXdPtr joint_damping_matrix_;
	Vector6dPtr control_point_damping_matrix_;

	VectorXdPtr joint_velocity_;
	VectorXdPtr joint_velocity_sum_;
	VectorXdPtr joint_torque_sum_;
	VectorXdPtr joint_velocity_command_;
	VectorXdPtr joint_total_velocity_;
	VectorXdPtr joint_total_torque_;

	VectorXdPtr joint_current_position_;
	VectorXdPtr joint_target_position_;
	VectorXdPtr joint_external_torque_;

	Vector6dPtr control_point_velocity_;
	Vector6dPtr control_point_velocity_sum_;
	Vector6dPtr control_point_force_sum_;
	Vector6dPtr control_point_velocity_command_;
	Vector6dPtr control_point_total_velocity_;
	Vector6dPtr control_point_total_force_;

	Vector6dPtr control_point_current_pose_;
	Vector6dPtr control_point_target_pose_;
	Vector6dPtr control_point_current_velocity_;
	Vector6dPtr control_point_current_acceleration_;
	Vector6dPtr control_point_external_force_;

	doublePtr scaling_factor_;

	MatrixXdPtr jacobian_;
	MatrixXdPtr jacobian_inverse_;
	Matrix4dPtr transformation_matrix_;
	Matrix6dPtr spatial_transformation_matrix_;
};

using RobotPtr = std::shared_ptr<Robot>;
using RobotConstPtr = std::shared_ptr<const Robot>;

}
