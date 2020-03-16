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

//! \file task_emergency_stop_constraint.h
//! \author Benjamin Navarro
//! \brief A constraint to stop the robot when a contant occurs.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

#include <physical_quantities/scalar/force.h>

namespace phri {

//! brief A constraint to stop the robot when a contant occurs.
//! details Two thresholds are used. The robot is stopped when the external
//! force is above the activation threshold and released when the external force
//! gets below the deactivation threshold.
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
    TaskEmergencyStopConstraint(
        std::shared_ptr<scalar::Force> activation_threshold,
        std::shared_ptr<scalar::Force> deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given referenced values
    //! \param activation_threshold A reference to the activation threshold
    //! (N). Make sure that \p activation_threshold outlives the constraint
    //! \param deactivation_threshold A reference to the deactivation
    //! threshold (N). Make sure that \p deactivation_threshold outlives
    //! the constraint
    TaskEmergencyStopConstraint(scalar::Force& activation_threshold,
                                scalar::Force& deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given values
    //! \param activation_threshold The value of the activation threshold (N)
    //! \param deactivation_threshold The value of the deactivation threshold
    //! (N)
    TaskEmergencyStopConstraint(const scalar::Force& activation_threshold,
                                const scalar::Force& deactivation_threshold);

    //! \brief Construct a new TaskEmergencyStopConstraint object using
    //! the given values
    //! \param activation_threshold The value of the activation threshold (N)
    //! \param deactivation_threshold The value of the deactivation threshold
    //! (N)
    TaskEmergencyStopConstraint(scalar::Force&& activation_threshold,
                                scalar::Force&& deactivation_threshold);

    /***		Algorithm		***/
    virtual double compute() override;

    scalar::Force& activationThreshold();
    scalar::Force activationThreshold() const;
    std::shared_ptr<scalar::Force> activationThresholdPtr() const;

    scalar::Force& deactivationThreshold();
    scalar::Force deactivationThreshold() const;
    std::shared_ptr<scalar::Force> deactivationThresholdPtr() const;

private:
    std::shared_ptr<scalar::Force> activation_threshold_;
    std::shared_ptr<scalar::Force> deactivation_threshold_;

    double previous_constraint_value_;
};

} // namespace phri
