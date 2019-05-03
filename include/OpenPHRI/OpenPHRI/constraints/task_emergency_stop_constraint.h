/*      File: stop_constraint.h
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
 * @file stop_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the JointEmergencyStopConstraint class
 * @date April 2017
 * @ingroup phri
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

/** @brief A constraint to stop the robot when a contant occurs.
 *  @details Two thresholds are used. The robot is stopped when the external
 * force is above the activation threshold and released when the external force
 * gets below the deactivation threshold.
 */
class TaskEmergencyStopConstraint : public Constraint {
public:
    //! \brief Construct a new TaskEmergencyStopConstraint object with an
    //! initial thresholds set to zero.
    //! \details Use TaskEmergencyStopConstraint::activationThreshold() and
    //! TaskEmergencyStopConstraint::deactivationThreshold() to set them to
    //! the desired value
    TaskEmergencyStopConstraint();

    //! \brief Construct a new TaskEmergencyStopConstraint object using the
    //! given pointed values
    //! \param activation_threshold A shared pointer to the activation
    //! threshold (N). Throws if the pointer is empty.
    //! \param deactivation_threshold A shared pointer to the deactivation
    //! threshold (N). Throws if the pointer is empty.
    TaskEmergencyStopConstraint(std::shared_ptr<double> activation_threshold,
                                std::shared_ptr<double> deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given referenced values
    //! \param activation_threshold A reference to the activation threshold
    //! (N). Make sure that \p activation_threshold outlives the constraint
    //! \param deactivation_threshold A reference to the deactivation
    //! threshold (N). Make sure that \p deactivation_threshold outlives
    //! the constraint
    TaskEmergencyStopConstraint(double& activation_threshold,
                                double& deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given values
    //! \param activation_threshold The value of the activation threshold (N)
    //! \param deactivation_threshold The value of the deactivation threshold
    //! (N)
    TaskEmergencyStopConstraint(const double& activation_threshold,
                                const double& deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given values
    //! \param activation_threshold The value of the activation threshold (N)
    //! \param deactivation_threshold The value of the deactivation threshold
    //! (N)
    TaskEmergencyStopConstraint(double&& activation_threshold,
                                double&& deactivation_threshold);

    //! \brief Default copy constructor
    TaskEmergencyStopConstraint(const TaskEmergencyStopConstraint&) = default;

    //! \brief Default move constructor
    TaskEmergencyStopConstraint(TaskEmergencyStopConstraint&&) = default;

    //! \brief Default virtual destructor
    virtual ~TaskEmergencyStopConstraint() = default;

    //! \brief Default copy operator
    TaskEmergencyStopConstraint&
    operator=(const TaskEmergencyStopConstraint&) = default;

    //! \brief Default move operator
    TaskEmergencyStopConstraint&
    operator=(TaskEmergencyStopConstraint&&) = default;

    /***		Algorithm		***/
    virtual double compute() override;

    double& activationThreshold();
    double activationThreshold() const;
    std::shared_ptr<double> activationThresholdPtr();

    double& deactivationThreshold();
    double deactivationThreshold() const;
    std::shared_ptr<double> deactivationThresholdPtr();

private:
    std::shared_ptr<double> activation_threshold_;
    std::shared_ptr<double> deactivation_threshold_;

    double previous_constraint_value_;
};

} // namespace phri
