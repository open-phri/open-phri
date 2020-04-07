/*      File: velocity_constraint.h
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

//! \file velocity_constraint.h
//! \author Benjamin Navarro
//! \brief A constraint to limit the TCP velocity.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/scalar/velocity.h>

namespace phri {

//! \brief A constraint to limit the TCP velocity.
class VelocityConstraint : public Constraint {
public:
    //! \brief Construct a new VelocityConstraint object with an initial
    //! limit set to zero.
    //! \details Use VelocityConstraint::maximumVelocity() to
    //! set it to the desired value
    VelocityConstraint();

    //! \brief Construct a new VelocityConstraint object using the given
    //! scalar::Velocity value, reference or (shared) pointer
    //!
    //! If maximum_velocity is a const reference/pointer, using
    //! maximumVelocity() to modify it will result in undefined behavior
    //!
    //! \tparam VmaxT The type of the value (automatically deduced)
    //! \param value The desired maximum velocity (m/s)
    template <typename VmaxT>
    explicit VelocityConstraint(VmaxT&& maximum_velocity) noexcept
        : maximum_velocity_{std::forward<VmaxT>(maximum_velocity)} {
    }

    //! \brief Compute the velocity constraint based on the robot state
    //! \return double The constraint value [0,1]
    [[nodiscard]] double compute() override;

    //! \brief Read/write access the velocity limit used by the constraint
    //! \return double& A reference to the velocity limit
    void setMaximumVelocity(const scalar::Velocity& velocity);

    //! \brief Read access the velocity limit used by the constraint
    //! \return double The velocity limit value
    [[nodiscard]] const scalar::Velocity& getMaximumVelocity() const;

protected:
    detail::UniversalWrapper<scalar::Velocity> maximum_velocity_;
};

} // namespace phri
