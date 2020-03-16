/*      File: joint_velocity_constraint.h
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

//! \file joint_velocity_constraint.h
//! \author Benjamin Navarro
//! \brief A constraint to limit the joint velocities
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/vector/velocity.h>

namespace phri {

//! \brief A constraint to limit the joint velocities.
class JointVelocityConstraint : public Constraint {
public:
    //! \brief Construct a new JointVelocityConstraint object with an initial
    //! limit set to zero.
    //! \details Use JointVelocityConstraint::maximumvelocity() to
    //! set it to the desired value
    JointVelocityConstraint();

    //! \brief Construct a new JointVelocityConstraint object using the given
    //! scalar::Velocity value, reference or (shared) pointer
    //!
    //! If maximum_velocity is a const reference/pointer, using
    //! maximumVelocity() to modify it will result in undefined behavior
    //!
    //! \tparam VmaxT The type of the value (automatically deduced)
    //! \param value The desired maximum velocity (m/s, rad/s)
    template <typename VmaxT>
    explicit JointVelocityConstraint(VmaxT&& maximum_velocities) noexcept
        : maximum_velocities_{std::forward<VmaxT>(maximum_velocities)} {
    }

    virtual double compute() override;

    //! \brief Read/write access the velocity limit used by the constraint
    //! \return double& A reference to the velocity limit
    vector::dyn::Velocity& maximumVelocities();

    //! \brief Read access the velocity limit used by the constraint
    //! \return double The velocity limit value
    const vector::dyn::Velocity& maximumVelocities() const;

protected:
    virtual void setRobot(Robot const* robot) override;

private:
    detail::UniversalWrapper<vector::dyn::Velocity> maximum_velocities_;
};

} // namespace phri
