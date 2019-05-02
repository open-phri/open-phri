/*      File: robot.h
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

/**
 * @file robot.h
 * @author Benjamin Navarro
 * @brief Definition of the Robot class
 * @date May 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

/** @brief A robot with all its state and control parameters
 */
class Robot {
public:
    //! \brief Holds a joint state (either current, target or command)
    struct JointData {
        //! \brief Construct a JointData with zero sized vectors
        JointData() = default;

        //! \brief Construct a JointData with \p joint_count sized vectors
        //! \see JointData::resize()
        JointData(size_t joint_count) {
            resize(joint_count);
        }

        //! \brief resize all the vectors to \p joint_count and fill them with
        //! zeros
        void resize(size_t joint_count) {
            position.resize(joint_count);
            velocity.resize(joint_count);
            acceleration.resize(joint_count);
            force.resize(joint_count);

            position.setZero();
            velocity.setZero();
            acceleration.setZero();
            force.setZero();
        }

        //! \brief Joints position (rad or m)
        phri::VectorXd position;
        //! \brief Joints velocity (rad/s or m/s)
        phri::VectorXd velocity;
        //! \brief Joints acceleration (rad/s² or m/s²)
        phri::VectorXd acceleration;
        //! \brief Joints force (Nm or N)
        phri::VectorXd force;
    };

    //! \brief Holds joints mechanical limits
    struct JointLimits {
        //! \brief Construct a JointLimits with zero sized vectors
        JointLimits() = default;

        //! \brief Construct a JointLimits with \p joint_count sized vectors
        //! \see JointLimits::resize()
        JointLimits(size_t joint_count) {
            resize(joint_count);
        }

        //! \brief resize all the vectors to \p joint_count and fill them with
        //! + or - infinity
        void resize(size_t joint_count) {
            min_position.resize(joint_count);
            max_position.resize(joint_count);
            max_velocity.resize(joint_count);
            max_acceleration.resize(joint_count);
            max_force.resize(joint_count);

            min_position.setConstant(-std::numeric_limits<double>::infinity());
            max_position.setConstant(std::numeric_limits<double>::infinity());
            max_velocity.setConstant(std::numeric_limits<double>::infinity());
            max_acceleration.setConstant(
                std::numeric_limits<double>::infinity());
            max_force.setConstant(std::numeric_limits<double>::infinity());
        }

        //! \brief Joints minimum position (rad or m)
        phri::VectorXd min_position;
        //! \brief Joints maximum position (rad or m)
        phri::VectorXd max_position;
        //! \brief Joints minimum velocity (rad/s or m/s)
        phri::VectorXd max_velocity;
        //! \brief Joints minimum acceleration (rad/s² or m/s²)
        phri::VectorXd max_acceleration;
        //! \brief Joints minimum force (Nm or N)
        phri::VectorXd max_force;
    };

    //! \brief Pack all joint related data (state, target, command, limits)
    struct Joints {
        //! \brief Construct a Joints with zero sized vectors
        Joints() = default;

        //! \brief Construct a Joints with \p joint_count sized vectors
        //! \see JointLimits::resize()
        Joints(size_t joint_count) {
            resize(joint_count);
        }

        //! \brief Call JointData::resize() on Joints::state, Joints::target and
        //! Joints::command and JointLimits::resize() on limits
        void resize(size_t joint_count) {
            state.resize(joint_count);
            target.resize(joint_count);
            command.resize(joint_count);
            limits.resize(joint_count);
        }

        //! \brief Joints current state (read from the robot)
        JointData state;
        //! \brief Joints targets (desired state)
        JointData target;
        //! \brief Joints commands (sent to the robot)
        JointData command;
        //! \brief Joints mechanical limits
        JointLimits limits;
    };

    //! \brief Holds a task state (either current, target or command)
    struct TaskData {
        //! \brief Task pose (m, rad)
        phri::Pose pose;
        //! \brief Task velocity (m/s, rad/s)
        phri::Twist twist;
        //! \brief Task acceleration (m/s², rad/s²)
        phri::Acceleration acceleration;
        //! \brief Task force (N, Nm)
        phri::Wrench wrench;
    };

    //! \brief Holds task physical limits
    struct TaskLimits {
        TaskLimits() {
            min_pose.translation().setConstant(
                -std::numeric_limits<double>::infinity());
            max_pose.translation().setConstant(
                std::numeric_limits<double>::infinity());
            max_twist.vector().setConstant(
                std::numeric_limits<double>::infinity());
            max_acceleration.vector().setConstant(
                std::numeric_limits<double>::infinity());
            max_wrench.vector().setConstant(
                std::numeric_limits<double>::infinity());
        }

        //! \brief Task minimum pose (m, rad)
        phri::Pose min_pose;
        //! \brief Task maximum pose (m, rad)
        phri::Pose max_pose;
        //! \brief Task maximum velocity (m/s, rad/s)
        phri::Twist max_twist;
        //! \brief Task maximum acceleration (m/s², rad/s²)
        phri::Acceleration max_acceleration;
        //! \brief Task maximum force (N, Nm)
        phri::Wrench max_wrench;
    };

    //! \brief Pack all task related data (state, target, command, limits)
    struct Task {
        //! \brief Task current state (read from the robot or set from the
        //! kinematic)
        TaskData state;
        //! \brief Task targets (desired state)
        TaskData target;
        //! \brief Task commands (sent to the robot, directly or after
        //! transformation to joint space)
        TaskData command;
        //! \brief Task physical limits
        TaskLimits limits;
    };

    //! \brief Holds control related data. Mainly used by the SafetyController
    struct ControlData {
        //! \brief Construct a ControlData with no joints
        ControlData() = default;

        //! \brief Construct a ControlData with \p joint_count joints
        //! \see ControlData::resize()
        ControlData(size_t joint_count) {
            resize(joint_count);
        }

        //! \brief Call ControlData::Joints::resize() on Joints::joints and
        //! resize ControlData::jacobian to a 6 x \p joint_count matrix and
        //! ControlData::jacobian_inverse to a \p joint_count x 6 matrix
        void resize(size_t joint_count) {
            joints.resize(joint_count);
            jacobian.resize(6, joint_count);
            jacobian_inverse.resize(joint_count, 6);
        }

        //! \brief Holds joint control related data
        struct Joints {
            //! \brief Construct a Joints with zero sized vectors
            Joints() = default;

            //! \brief Construct a Joints with \p joint_count sized vectors
            //! \see Joints::resize()
            Joints(size_t joint_count) {
                resize(joint_count);
            }

            //! \brief resize all the vectors to \p joint_count and fill them
            //! with zeros, except for Joints::damping which is set to infinity
            void resize(size_t joint_count) {
                damping.resize(joint_count);
                velocity_sum.resize(joint_count);
                force_sum.resize(joint_count);
                velocity_command.resize(joint_count);
                total_velocity.resize(joint_count);
                total_force.resize(joint_count);

                damping.setConstant(std::numeric_limits<double>::max());
                velocity_sum.setZero();
                force_sum.setZero();
                velocity_command.setZero();
                total_velocity.setZero();
                total_force.setZero();
            }

            //! \brief Damping matrix used by the joint-level damping
            //! control
            phri::VectorXd damping;
            //! \brief Sum of all joint velocity inputs
            phri::VectorXd velocity_sum;
            //! \brief Sum of all joint force inputs
            phri::VectorXd force_sum;
            //! \brief Output of the joint-level admittance controller
            phri::VectorXd velocity_command;
            //! \brief Cumulative effect on the joint velocity of all inputs.
            //! \details Same as joints.command.velocity but without the scaling
            //! factor.
            phri::VectorXd total_velocity;
            //! \brief Cumulative effect of both joint torque and control point
            //! force inputs.
            phri::VectorXd total_force;
        };

        //! \brief Holds task control related data
        struct Task {
            Task() {
                damping.setConstant(std::numeric_limits<double>::max());
                twist_sum.vector().setZero();
                wrench_sum.vector().setZero();
                twist_command.vector().setZero();
                total_twist.vector().setZero();
                total_wrench.vector().setZero();
            }

            //! \brief Damping matrix used by the task-level damping control
            phri::Vector6d damping;
            //! \brief Sum of all task velocity inputs
            phri::Twist twist_sum;
            //! \brief Sum of all task force inputs
            phri::Wrench wrench_sum;
            //! \brief Output of the task-level admittance controller
            phri::Twist twist_command;
            //! \brief Cumulative effect on the task velocity of all inputs.
            //! \details Same as task.command.velocity but without the scaling
            //! factor.
            phri::Twist total_twist;
            //! \brief Cumulative effect of both joint torque and control point
            //! force inputs.
            phri::Wrench total_wrench;
        };

        //! \brief Joint control related data
        Joints joints;

        //! \brief task control related data
        Task task;

        //! \brief Velocity scaling factor. Computed by the SafetyController to
        //! ensure the constraints
        double scaling_factor;

        //! \brief The time step, in seconds, used to control the robot
        double time_step;

        //! \brief Task Jacobian matrix
        phri::MatrixXd jacobian;

        //! \brief Task Jacobian (pseudo-)matrix
        phri::MatrixXd jacobian_inverse;

        //! \brief Task transformation matrix (R T;0 1)
        phri::Matrix4d transformation_matrix;

        //! \brief Task spatial transformation matrix ([R 0;0 R])
        phri::Matrix6d spatial_transformation_matrix;
    };

    //! \brief Construct a new Robot object with no name and zero joints. In
    //! this case, Robot::create() must be called before use
    Robot() = default;

    //! \brief Construct a new Robot object with a name and a number of joints
    //! \param name The name of the robot
    //! \param joint_count The number of joints
    Robot(const std::string& name, size_t joint_count);

    //! \brief Construct a new Robot object with the given configuration
    //! \details The \p configuration node must contain a \a name field and a \a
    //! joint_count field
    //! \param configuration A YAML::Node describing the robot
    Robot(const YAML::Node& configuration);

    //! \brief Change the name and number of joints of the robot
    //! \param name The name of the robot
    //! \param joint_count The number of joints
    void create(const std::string& name, size_t joint_count);

    //! \brief Change the name and number of joints of the robot according to
    //! the given configuration
    //! \details The \p configuration node must contain
    //! a \a name field (string) and a \a joint_count (unsigned int) field
    //! \param configuration A YAML::Node describing the robot
    void create(const YAML::Node& configuration);

    //! \brief The name given to the robot.
    //! \return A const ref to the name.
    const std::string& name() const;

    //! \brief Number of joints of the robot.
    //! \return The number of joints.
    size_t jointCount() const;

    //! \brief The robot's joints data
    Joints joints;

    //! \brief The robot's task data
    Task task;

    //! \brief Control related data
    ControlData control;

private:
    std::string name_;
    size_t joint_count_;
};

using RobotPtr = std::shared_ptr<Robot>;
using RobotConstPtr = std::shared_ptr<const Robot>;

} // namespace phri
