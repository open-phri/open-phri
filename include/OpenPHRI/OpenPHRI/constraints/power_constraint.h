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
#include <OpenPHRI/detail/universal_wrapper.hpp>

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
    //! scalar::Power value, reference or (shared) pointer
    //!
    //! If maximum_power is a const reference/pointer, using
    //! maximumPower() to modify it will result in undefined behavior
    //!
    //! \tparam PmaxT The type of the value (automatically deduced)
    //! \param value The desired maximum power (m/s)
    template <typename PmaxT>
    explicit PowerConstraint(PmaxT&& maximum_power) noexcept
        : maximum_power_{std::forward<PmaxT>(maximum_power)} {
    }

    //! \brief Compute the power constraint based on the robot state
    //! \return double The constraint value [0,1]
    virtual double compute() override;

    //! \brief Read/write access the power limit used by the constraint
    //! \return double& A reference to the power limit
    void setMaximumPower(const scalar::Power& power);

    //! \brief Read access the power limit used by the constraint
    //! \return double The power limit value
    const scalar::Power& getMaximumPower() const;

    //! \brief Read access the current exchanged power
    //! \return double The exchanged power
    const scalar::Power& getPower() const;

private:
    //! \brief The power limit.
    detail::UniversalWrapper<scalar::Power> maximum_power_;

    //! \brief Currently exchanged power.
    scalar::Power power_;
};

} // namespace phri
