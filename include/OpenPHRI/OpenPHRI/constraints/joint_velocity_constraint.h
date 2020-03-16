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
    //! pointed value
    //! \param maximum_velocities A shared pointer to the desired
    //! maximum velocity (rad/s, m/s). Throws if the pointer is empty.
    explicit JointVelocityConstraint(
        std::shared_ptr<vector::dyn::Velocity> maximum_velocities);

    //! \brief Construct a new JointVelocityConstraint object using the given
    //! referenced value
    //! \param maximum_velocities A reference to the desired
    //! maximum velocity (rad/s, m/s). Make sure that \p maximum_velocities
    //! outlives the constraint
    explicit JointVelocityConstraint(vector::dyn::Velocity& maximum_velocities);

    //! \brief Construct a new JointVelocityConstraint object using the given
    //! value
    //! \param maximum_velocities The value of the desired maximum
    //! velocity (rad/s, m/s). Use JointVelocityConstraint::maximumvelocity() to
    //! update the limit
    explicit JointVelocityConstraint(
        const vector::dyn::Velocity& maximum_velocities);

    //! \brief Construct a new JointVelocityConstraint object using the given
    //! value
    //! \param maximum_velocities The value of the desired maximum
    //! velocity (rad/s, m/s). Use JointVelocityConstraint::maximumvelocity() to
    //! update the limit
    explicit JointVelocityConstraint(
        vector::dyn::Velocity&& maximum_velocities);

    virtual double compute() override;

    //! \brief Read/write access the velocity limit used by the constraint
    //! \return double& A reference to the velocity limit
    vector::dyn::Velocity& maximumVelocities();

    //! \brief Read access the velocity limit used by the constraint
    //! \return double The velocity limit value
    const vector::dyn::Velocity& maximumVelocities() const;

    //! \brief Access to the shared pointer holding the velocity limit used
    //! by the constraint
    //! \return std::shared_ptr<double> A shared pointer to the velocity
    //! limit
    std::shared_ptr<vector::dyn::Velocity> maximumVelocitiesPtr() const;

protected:
    virtual void setRobot(Robot const* robot) override;

private:
    std::shared_ptr<vector::dyn::Velocity> maximum_velocities_;
};

} // namespace phri
