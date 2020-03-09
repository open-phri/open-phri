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
#include <physical_quantities/spatial/position.h>
#include <physical_quantities/spatial/velocity.h>
#include <physical_quantities/spatial/acceleration.h>
#include <physical_quantities/spatial/force.h>
#include <physical_quantities/spatial/impedance/damping.h>
#include <physical_quantities/spatial/transformation.h>

namespace phri {

/** @brief A robot with all its state and control parameters
 */
class Robot {
public:
    //! \brief Holds a joint state (either current, target or command)
    class JointData {
    public:
        //! \brief Construct a JointData with zero sized vectors
        JointData() = default;

        //! \brief Construct a JointData with \p joint_count sized vectors
        //! \see JointData::resize()
        explicit JointData(size_t joint_count);

        //! \brief resize all the vectors to \p joint_count and fill them with
        //! zeros
        void resize(size_t joint_count);

        //! \brief Joints position (rad or m)
        Eigen::VectorXd& position();
        const Eigen::VectorXd& position() const;
        //! \brief Joints velocity (rad/s or m/s)
        Eigen::VectorXd& velocity();
        const Eigen::VectorXd& velocity() const;
        //! \brief Joints acceleration (rad/s² or m/s²)
        Eigen::VectorXd& acceleration();
        const Eigen::VectorXd& acceleration() const;
        //! \brief Joints force (Nm or N)
        Eigen::VectorXd& force();
        const Eigen::VectorXd& force() const;

    private:
        Eigen::VectorXd position_;
        Eigen::VectorXd velocity_;
        Eigen::VectorXd acceleration_;
        Eigen::VectorXd force_;
    };

    //! \brief Holds joints mechanical limits
    class JointLimits {
    public:
        //! \brief Construct a JointLimits with zero sized vectors
        JointLimits() = default;

        //! \brief Construct a JointLimits with \p joint_count sized vectors
        //! \see JointLimits::resize()
        JointLimits(size_t joint_count);

        //! \brief resize all the vectors to \p joint_count and fill them with
        //! + or - infinity
        void resize(size_t joint_count);

        //! \brief Joints minimum position (rad or m)
        Eigen::Ref<const Eigen::VectorXd> minPosition() const;
        //! \brief Joints maximum position (rad or m)
        Eigen::Ref<const Eigen::VectorXd> maxPosition() const;
        //! \brief Joints minimum velocity (rad/s or m/s)
        Eigen::Ref<const Eigen::VectorXd> maxVelocity() const;
        //! \brief Joints minimum acceleration (rad/s² or m/s²)
        Eigen::Ref<const Eigen::VectorXd> maxAcceleration() const;
        //! \brief Joints minimum force (Nm or N)
        Eigen::Ref<const Eigen::VectorXd> maxForce() const;

        mutable double safety_factor;

    private:
        friend class RobotModel;
        Eigen::VectorXd min_position_;
        Eigen::VectorXd max_position_;
        Eigen::VectorXd max_velocity_;
        Eigen::VectorXd max_acceleration_;
        Eigen::VectorXd max_force_;
    };

    //! \brief Pack all joint related data (state, target, command, limits)
    class Joints {
    public:
        //! \brief Construct a Joints with zero sized vectors
        Joints() = default;

        //! \brief Construct a Joints with \p joint_count sized vectors
        //! \see JointLimits::resize()
        explicit Joints(size_t joint_count);

        //! \brief Call JointData::resize() on Joints::state, Joints::target and
        //! Joints::command and JointLimits::resize() on limits
        void resize(size_t joint_count);

        //! \brief Joints current state (read from the robot)
        const JointData& state() const;

        //! \brief Joints targets (desired state)
        JointData& target();
        const JointData& target() const;

        //! \brief Joints commands (sent to the robot)
        const JointData& command() const;

        //! \brief Joints mechanical limits
        const JointLimits& limits() const;

    private:
        friend class SafetyController;
        friend class RobotModel;
        friend class Driver;

        JointData state_;
        JointData target_;
        JointData command_;
        JointLimits limits_;
    };

    //! \brief Holds a task state (either current, target or command)
    class TaskData {
    public:
        TaskData(spatial::Frame frame);

        //! \brief Task pose (m, rad)
        spatial::Position& position();
        const spatial::Position& position() const;

        //! \brief Task velocity (m/s, rad/s)
        spatial::Velocity& velocity();
        const spatial::Velocity& velocity() const;

        //! \brief Task acceleration (m/s², rad/s²)
        spatial::Acceleration& acceleration();
        const spatial::Acceleration& acceleration() const;

        //! \brief Task force (N, Nm)
        spatial::Force& force();
        const spatial::Force& force() const;

    private:
        spatial::Position position_;
        spatial::Velocity velocity_;
        spatial::Acceleration acceleration_;
        spatial::Force wrench_;
    };

    //! \brief Holds task physical limits
    class TaskLimits {
    public:
        TaskLimits(spatial::Frame frame);

        //! \brief Task minimum pose (m, rad)
        spatial::Position& minPosition();
        const spatial::Position& minPosition() const;

        //! \brief Task maximum pose (m, rad)
        spatial::Position& maxPosition();
        const spatial::Position& maxPosition() const;

        //! \brief Task maximum velocity (m/s, rad/s)
        spatial::Velocity& maxVelocity();
        const spatial::Velocity& maxVelocity() const;

        //! \brief Task maximum acceleration (m/s², rad/s²)
        spatial::Acceleration& maxAcceleration();
        const spatial::Acceleration& maxAcceleration() const;

        //! \brief Task maximum force (N, Nm)
        spatial::Force& maxForce();
        const spatial::Force& maxForce() const;

    private:
        spatial::Position min_position_;
        spatial::Position max_position_;
        spatial::Velocity max_velocity_;
        spatial::Acceleration max_acceleration_;
        spatial::Force max_force_;
    };

    //! \brief Pack all task related data (state, target, command, limits)
    class Task {
    public:
        explicit Task(spatial::Frame frame);

        //! \brief Task current state (read from the robot or set from the
        //! kinematic)
        const TaskData& state() const;

        //! \brief Task targets (desired state)
        const TaskData& target() const;
        TaskData& target();

        //! \brief Task commands (sent to the robot, directly or after
        //! transformation to joint space)
        const TaskData& command() const;

        //! \brief Task physical limits
        const TaskLimits& limits() const;

    private:
        friend class SafetyController;
        friend class RobotModel;
        friend class Driver;

        TaskData state_;
        TaskData target_;
        TaskData command_;
        TaskLimits limits_;
    };

    //! \brief Holds control related data. Mainly used by the SafetyController
    struct ControlData {
        //! \brief Construct a ControlData with no joints
        ControlData(spatial::Frame frame, spatial::Frame parent);

        //! \brief Construct a ControlData with \p joint_count joints
        //! \see ControlData::resize()
        ControlData(spatial::Frame frame, spatial::Frame parent,
                    size_t joint_count);

        //! \brief Call ControlData::Joints::resize() on Joints::joints and
        //! resize ControlData::jacobian to a 6 x \p joint_count matrix and
        //! ControlData::jacobian_inverse to a \p joint_count x 6 matrix
        void resize(size_t joint_count);

        //! \brief Holds joint control related data
        class Joints {
        public:
            //! \brief Construct a Joints with zero sized vectors
            Joints() = default;

            //! \brief Construct a Joints with \p joint_count sized vectors
            //! \see Joints::resize()
            Joints(size_t joint_count);

            //! \brief resize all the vectors to \p joint_count and fill them
            //! with zeros, except for Joints::damping which is set to infinity
            void resize(size_t joint_count);

            //! \brief Damping matrix used by the joint-level damping
            //! control
            Eigen::VectorXd& damping();
            const Eigen::VectorXd& damping() const;
            //! \brief Sum of all joint velocity inputs
            const Eigen::VectorXd& velocitySum() const;

            //! \brief Sum of all joint force inputs
            const Eigen::VectorXd& forceSum() const;

            //! \brief Output of the joint-level admittance controller
            const Eigen::VectorXd& velocityCommand() const;

            //! \brief Cumulative effect on the joint velocity of all inputs.
            //! \details Same as joints.command.velocity but without the scaling
            //! factor.
            const Eigen::VectorXd& totalVelocity() const;

            //! \brief Cumulative effect of both joint torque and control point
            //! force inputs.
            const Eigen::VectorXd& totalForce() const;

        private:
            friend class SafetyController;
            Eigen::VectorXd damping_;
            Eigen::VectorXd velocity_sum_;
            Eigen::VectorXd force_sum_;
            Eigen::VectorXd velocity_command_;
            Eigen::VectorXd total_velocity_;
            Eigen::VectorXd total_force_;
        };

        //! \brief Holds task control related data
        class Task {
        public:
            Task(spatial::Frame frame);

            //! \brief Damping matrix used by the task-level damping control
            spatial::Damping& damping();
            const spatial::Damping& damping() const;

            //! \brief Sum of all task velocity inputs
            const spatial::Velocity& velocitySum() const;

            //! \brief Sum of all task force inputs
            const spatial::Force& forceSum() const;

            //! \brief Output of the task-level admittance controller
            const spatial::Velocity& velocityCommand() const;

            //! \brief Cumulative effect on the task velocity of all inputs.
            //! \details Same as task.command.velocity but without the scaling
            //! factor.
            const spatial::Velocity& totalVelocity() const;

            //! \brief Cumulative effect of both joint torque and control point
            //! force inputs.
            const spatial::Force& totalForce() const;

        private:
            friend class SafetyController;
            spatial::Damping damping_;
            spatial::Velocity velocity_sum_;
            spatial::Force force_sum_;
            spatial::Velocity velocity_command_;
            spatial::Velocity total_velocity_;
            spatial::Force total_force_;
        };

        //! \brief Joint control related data
        Joints& joints();
        const Joints& joints() const;

        //! \brief task control related data
        Task& task();
        const Task& task() const;

        //! \brief Velocity scaling factor. Computed by the SafetyController to
        //! ensure the constraints
        const double& scalingFactor() const;

        //! \brief The time step, in seconds, used to control the robot
        const double& timeStep() const;

        //! \brief Task Jacobian matrix
        const Eigen::MatrixXd& jacobian() const;

        //! \brief Task Jacobian (pseudo-)matrix
        const Eigen::MatrixXd& jacobianInverse() const;

        //! \brief Task transformation matrix (R T;0 1)
        const spatial::Transformation& transformation() const;

    private:
        friend class SafetyController;
        friend class Driver;
        friend class RobotModel;

        //! \brief Joint control related data
        Joints joints_;

        //! \brief task control related data
        Task task_;

        //! \brief Velocity scaling factor. Computed by the SafetyController to
        //! ensure the constraints
        double scaling_factor_;

        //! \brief The time step, in seconds, used to control the robot
        double time_step_;

        //! \brief Task Jacobian matrix
        Eigen::MatrixXd jacobian_;

        //! \brief Task Jacobian (pseudo-)matrix
        Eigen::MatrixXd jacobian_inverse_;

        //! \brief Task transformation matrix (R T;0 1)
        spatial::Transformation transformation_;
    };

    Robot(spatial::Frame control_point_frame,
          spatial::Frame control_point_parent_frame);

    //! \brief Construct a new Robot object with a name and a number of joints
    //! \param name The name of the robot
    //! \param joint_count The number of joints
    Robot(spatial::Frame control_point_frame,
          spatial::Frame control_point_parent_frame, const std::string& name,
          size_t joint_count);

    //! \brief Construct a new Robot object with the given configuration
    //! \details The \p configuration node must contain a \a name field and a \a
    //! joint_count field
    //! \param configuration A YAML::Node describing the robot
    Robot(const YAML::Node& configuration);

    //! \brief Change the name and number of joints of the robot
    //! \param name The name of the robot
    //! \param joint_count The number of joints
    void create(const std::string& name, size_t joint_count);

    //! \brief Change the name, number of joints and controlled frame of the
    //! robot
    //! \param name The name of the robot
    //! \param joint_count The number of joints
    //! \param frame The controlled frame
    void create(const std::string& name, size_t joint_count,
                spatial::Frame control_point_frame,
                spatial::Frame control_point_parent_frame);

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

    //! \brief Frame associated with the control point
    //! \return The frame.
    spatial::Frame controlPointFrame() const;

    //! \brief Parent frame of the one associated with the control point
    //! \return The frame.
    spatial::Frame controlPointParentFrame() const;

    //! \brief The robot's joints data
    Joints& joints();
    const Joints& joints() const;

    //! \brief The robot's task data
    Task& task();
    const Task& task() const;

    //! \brief Control related data
    ControlData& control();
    const ControlData& control() const;

private:
    spatial::Frame control_point_frame_;
    spatial::Frame control_point_parent_frame_;
    std::string name_;
    size_t joint_count_;

    //! \brief The robot's joints data
    Joints joints_;

    //! \brief The robot's task data
    Task task_;

    //! \brief Control related data
    ControlData control_;
};

using RobotPtr = std::shared_ptr<Robot>;
using RobotConstPtr = std::shared_ptr<const Robot>;

} // namespace phri
