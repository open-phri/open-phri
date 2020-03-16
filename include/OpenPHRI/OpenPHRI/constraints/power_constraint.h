/*      File: power_constraint.h
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

//! \file power_constraint.h
//! \author Benjamin Navarro
//! \brief A constraint to limit the exchanged power
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

#include <physical_quantities/scalar/power.h>

namespace phri {

//! \brief A constraint to limit the exchanged power.
//! \details The power considered is the dot product of the total velocity with
//! the external force. The constraint is active only when the power is
//! negative, which is when the robot pushes the environment. No limitation is
//! applied when the robot is pushed by the environment.
class PowerConstraint : public Constraint {
public:
    //! \brief Construct a new PowerConstraint object with an initial
    //! limit set to zero.
    //! \details Use PowerConstraint::maximumPower() to
    //! set it to the desired value
    PowerConstraint();

    //! \brief Construct a new PowerConstraint object using the given
    //! pointed value
    //! \param maximum_power A shared pointer to the desired
    //! maximum power (W). Throws if the pointer is empty.
    explicit PowerConstraint(std::shared_ptr<scalar::Power> maximum_power);

    //! \brief Construct a new PowerConstraint object using the given
    //! referenced value
    //! \param maximum_power A reference to the desired
    //! maximum power (W). Make sure that \p maximum_power
    //! outlives the constraint
    explicit PowerConstraint(scalar::Power& maximum_power);

    //! \brief Construct a new PowerConstraint object using the given
    //! value
    //! \param maximum_power The value of the desired maximum
    //! power (W). Use PowerConstraint::maximumPower()
    //! to update the limit
    explicit PowerConstraint(const scalar::Power& maximum_power);

    //! \brief Construct a new PowerConstraint object using the given
    //! value
    //! \param maximum_power The value of the desired maximum
    //! power (W). Use PowerConstraint::maximumPower()
    //! to update the limit
    explicit PowerConstraint(scalar::Power&& maximum_power);

    //! \brief Compute the power constraint based on the robot state
    //! \return double The constraint value [0,1]
    virtual double compute() override;

    //! \brief Read/write access the power limit used by the constraint
    //! \return double& A reference to the power limit
    scalar::Power& maximumPower();

    //! \brief Read access the power limit used by the constraint
    //! \return double The power limit value
    scalar::Power maximumPower() const;

    //! \brief Access to the shared pointer holding the power limit used
    //! by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the power
    //! limit
    std::shared_ptr<scalar::Power> maximumPowerPtr() const;

    //! \brief Read access the current exchanged power
    //! \return double The exchanged power
    scalar::Power power() const;

    //! \brief Access to the shared pointer holding the current exchanged power
    //! \return std::shared_ptr<double> A shared pointer to the exchanged power
    std::shared_ptr<const scalar::Power> powerPtr() const;

private:
    //! \brief Shared pointer holding the power limit.
    std::shared_ptr<scalar::Power> maximum_power_;

    //! \brief Shared pointer the currently exchanged power.
    std::shared_ptr<scalar::Power> power_;
};

} // namespace phri
