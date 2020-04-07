/*      File: constraint.h
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

//! \file constraint.h
//! \author Benjamin Navarro
//! \brief Base class for all constraints
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

//! \brief Base class for all constraints
//! \details A constraint is a scalar value in [0, 1] that is applied on the
//! output of the SafetyController to limit the robot velocity when needed to
//! enforce safety limitations.
//!
//!  Derived classes just have to implement the Contraint::compute() method.
class Constraint {
public:
    //! \brief Default constructor
    Constraint() = default;

    //! \brief Delete copy constructor to avoid slicing
    Constraint(const Constraint&) = delete;

    //! \brief Default move constructor
    Constraint(Constraint&&) = default;

    //! \brief Default virtual destructor
    virtual ~Constraint() = default;

    //! \brief Delete copy operator to avoid slicing
    Constraint& operator=(const Constraint&) = delete;

    //! \brief Default move operator
    Constraint& operator=(Constraint&&) = default;

    //! \brief Compute the value associated with the constraint
    //! \return double The constraint's evaluated value.
    virtual double compute() = 0;

    //! \brief Call operator, shortcut for Constraint::compute()
    //! \return double \see Constraint::compute()
    double operator()();

protected:
    friend class SafetyController;
    template <typename ConstraintT, typename InterpolatorT>
    friend class SeparationDistanceConstraint;

    //! \brief Set the robot to work with.
    //! \param robot The robot.
    virtual void setRobot(Robot const* robot);

    //! \brief Read/write access the controlled robot
    //! \return double& A reference to the controlled robot
    const Robot& robot() const;

    //! \brief The robot on which the constraint is applied.
    //! \details This will automatically set by the SafetyController or a
    //! SeparationDistanceConstraint
    Robot const* robot_{nullptr};
};

} // namespace phri
