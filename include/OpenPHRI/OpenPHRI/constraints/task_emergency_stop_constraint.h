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
#include <OpenPHRI/detail/universal_wrapper.hpp>

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
    //! given scalar::Force values, references or (shared) pointers
    //!
    //! If either activation_threshold/deactivation_threshold are a const
    //! references/pointers, using activationThreshold()/deactivationThreshold()
    //! to modify them will result in undefined behavior
    //!
    //! \tparam ActThT The type of the value (automatically deduced)
    //! \tparam DeactThT The type of the value (automatically deduced)
    //! \param activation_threshold The desired activation threshold (N)
    //! \param deactivation_threshold The desired deactivation threshold (N)
    template <typename ActThT, typename DeactThT>
    explicit TaskEmergencyStopConstraint(
        ActThT&& activation_threshold,
        DeactThT&& deactivation_threshold) noexcept
        : activation_threshold_{std::forward<ActThT>(activation_threshold)},
          deactivation_threshold_{
              std::forward<DeactThT>(deactivation_threshold)} {
    }

    /***		Algorithm		***/
    double compute() override;

    void setActivationThreshold(const scalar::Force& threshold);
    const scalar::Force& getActivationThreshold() const;

    void setDeactivationThreshold(const scalar::Force& threshold);
    const scalar::Force& getDeactivationThreshold() const;

private:
    detail::UniversalWrapper<scalar::Force> activation_threshold_;
    detail::UniversalWrapper<scalar::Force> deactivation_threshold_;

    double previous_constraint_value_;
};

} // namespace phri
