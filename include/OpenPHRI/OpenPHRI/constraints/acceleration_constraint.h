/*      File: acceleration_constraint.h
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

//! \file acceleration_constraint.h
//! \author Benjamin Navarro
//! \brief A phri::Constraint to limit the task space acceleration.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/constraint.h>

namespace phri {

//! \brief A phri::Constraint to limit the task space acceleration.
class AccelerationConstraint : public Constraint {
public:
    //! \brief Construct a new AccelerationConstraint object with an initial
    //! limit set to zero.
    //! \details Use AccelerationConstraint::maximumAcceleration() to
    //! set it to the desired value
    AccelerationConstraint();

    //! \brief Construct a new AccelerationConstraint object using the given
    //! pointed value
    //! \param maximum_acceleration A shared pointer to the desired
    //! maximum acceleration (m/s²). Throws if the pointer is empty.
    explicit AccelerationConstraint(
        std::shared_ptr<double> maximum_acceleration);

    //! \brief Construct a new AccelerationConstraint object using the given
    //! referenced value
    //! \param maximum_acceleration A reference to the desired
    //! maximum acceleration (m/s²). Make sure that \p maximum_acceleration
    //! outlives the constraint
    explicit AccelerationConstraint(double& maximum_acceleration);

    //! \brief Construct a new AccelerationConstraint object using the given
    //! value
    //! \param maximum_acceleration The value of the desired maximum
    //! acceleration (m/s²). Use AccelerationConstraint::maximumAcceleration()
    //! to update the limit
    explicit AccelerationConstraint(const double& maximum_acceleration);

    //! \brief Construct a new AccelerationConstraint object using the given
    //! value
    //! \param maximum_acceleration The value of the desired maximum
    //! acceleration (m/s²). Use AccelerationConstraint::maximumAcceleration()
    //! to update the limit
    explicit AccelerationConstraint(double&& maximum_acceleration);

    //! \brief Default copy constructor
    AccelerationConstraint(const AccelerationConstraint&) = default;

    //! \brief Default move constructor
    AccelerationConstraint(AccelerationConstraint&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref AccelerationConstraint::maximum_acceleration_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~AccelerationConstraint() = default;

    //! \brief Default copy operator
    AccelerationConstraint& operator=(const AccelerationConstraint&) = default;

    //! \brief Default move operator
    AccelerationConstraint& operator=(AccelerationConstraint&&) = default;

    //! \brief Compute the acceleration constraint based on the robot state
    //! \return double The constraint value [0,1]
    virtual double compute() override;

    //! \brief Read/write access the acceleration limit used by the constraint
    //! \return double& A reference to the acceleration limit
    double& maximumAcceleration();

    //! \brief Read access the acceleration limit used by the constraint
    //! \return double The acceleration limit value
    double maximumAcceleration() const;

    //! \brief Access to the shared pointer holding the acceleration limit used
    //! by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the acceleration
    //! limit
    std::shared_ptr<double> maximumAccelerationPtr() const;

private:
    //! \brief Shared pointer holding the acceleration limit.
    std::shared_ptr<double> maximum_acceleration_;
};

} // namespace phri
