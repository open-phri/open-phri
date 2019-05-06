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
    //! pointed value
    //! \param maximum_velocity A shared pointer to the desired
    //! maximum velocity (m/s). Throws if the pointer is empty.
    explicit VelocityConstraint(std::shared_ptr<double> maximum_velocity);

    //! \brief Construct a new VelocityConstraint object using the given
    //! referenced value
    //! \param maximum_velocity A reference to the desired
    //! maximum velocity (m/s). Make sure that \p maximum_velocity
    //! outlives the constraint
    explicit VelocityConstraint(double& maximum_velocity);

    //! \brief Construct a new VelocityConstraint object using the given
    //! value
    //! \param maximum_velocity The value of the desired maximum
    //! velocity (m/s). Use VelocityConstraint::maximumVelocity()
    //! to update the limit
    explicit VelocityConstraint(const double& maximum_velocity);

    //! \brief Construct a new VelocityConstraint object using the given
    //! value
    //! \param maximum_velocity The value of the desired maximum
    //! velocity (m/s). Use VelocityConstraint::maximumVelocity()
    //! to update the limit
    explicit VelocityConstraint(double&& maximum_velocity);

    //! \brief Default copy constructor
    VelocityConstraint(const VelocityConstraint&) = default;

    //! \brief Default move constructor
    VelocityConstraint(VelocityConstraint&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref VelocityConstraint::maximum_velocity_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~VelocityConstraint() = default;

    //! \brief Default copy operator
    VelocityConstraint& operator=(const VelocityConstraint&) = default;

    //! \brief Default move operator
    VelocityConstraint& operator=(VelocityConstraint&&) = default;

    //! \brief Compute the velocity constraint based on the robot state
    //! \return double The constraint value [0,1]
    virtual double compute() override;

    //! \brief Read/write access the velocity limit used by the constraint
    //! \return double& A reference to the velocity limit
    double& maximumVelocity();

    //! \brief Read access the velocity limit used by the constraint
    //! \return double The velocity limit value
    double maximumVelocity() const;

    //! \brief Access to the shared pointer holding the velocity limit used
    //! by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the velocity
    //! limit
    std::shared_ptr<double> maximumVelocityPtr() const;

protected:
    std::shared_ptr<double> maximum_velocity_;
};

} // namespace phri
