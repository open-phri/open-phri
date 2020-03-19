/*      File: joint_acceleration_constraint.h
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

//! \file joint_acceleration_constraint.h
//! \author Benjamin Navarro
//! \brief A phri::Constraint to limit the task space acceleration.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/vector/acceleration.h>

namespace phri {

//! \brief A phri::Constraint to limit the task space acceleration.
class JointAccelerationConstraint : public Constraint {
public:
    //! \brief Construct a new JointAccelerationConstraint object with an
    //! initial limit set to zero.
    //! \details Use JointAccelerationConstraint::maximumAcceleration() to set
    //! it to the desired value
    JointAccelerationConstraint();

    //! \brief Construct a new AccelerationConstraint object using the given
    //! vector::dyn::Acceleration value, reference or (shared) pointer
    //!
    //! If maximum_acceleration is a const reference/pointer, using
    //! maximumAcceleration() to modify it will result in undefined behavior
    //!
    //! \tparam AmaxT The type of the value (automatically deduced)
    //! \param value The desired maximum acceleration (m/s)
    template <typename AmaxT>
    explicit JointAccelerationConstraint(AmaxT&& maximum_acceleration) noexcept
        : maximum_acceleration_{std::forward<AmaxT>(maximum_acceleration)} {
    }

    //! \brief Compute the acceleration constraint based on the robot state
    //! \return double The constraint value [0,1]
    virtual double compute() override;

    //! \brief Read/write access the acceleration limit used by the constraint
    //! \return vector::dyn::Acceleration& A reference to the acceleration limit
    void setMaximumAcceleration(const vector::dyn::Acceleration& acceleration);

    //! \brief Read access the acceleration limit used by the constraint
    //! \return vector::dyn::Acceleration The acceleration limit value
    const vector::dyn::Acceleration& getMaximumAcceleration() const;

protected:
    virtual void setRobot(Robot const* robot) override;

private:
    detail::UniversalWrapper<vector::dyn::Acceleration> maximum_acceleration_;
};

} // namespace phri