/*      File: null_space_motion.h
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2018 -
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

//! \file null_space_motion.h
//! \author Benjamin Navarro
//! \brief Adds a joint velocity in the null space of the task jacobian.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

//! \brief Adds a joint velocity in the null space of the task jacobian.
//! \details Can be useful to perform a secondary task such as joint limits
//! avoidance.
class NullSpaceMotion : public JointVelocityGenerator {
public:
    //! \brief Construct a new NullSpaceMotion object with an initial
    //! velocity set to zero.
    //! \details Use NullSpaceMotion::velocity() to set it to the desired
    //! value
    NullSpaceMotion();

    //! \brief Construct a new NullSpaceMotion object using the given
    //! pointed value
    //! \param velocity The velocity to forward
    explicit NullSpaceMotion(std::shared_ptr<vector::dyn::Velocity> velocity);

    //! \brief Construct a new NullSpaceMotion object using the given
    //! referenced value
    //! \param velocity The velocity to forward. Make sure that \p velocity
    //! outlives the generator
    explicit NullSpaceMotion(vector::dyn::Velocity& velocity);

    //! \brief Construct a new NullSpaceMotion object using the given
    //! value
    //! \param velocity The velocity to forward
    explicit NullSpaceMotion(const vector::dyn::Velocity& velocity);

    //! \brief Construct a new NullSpaceMotion object using the given
    //! value
    //! \param velocity The velocity to forward
    explicit NullSpaceMotion(vector::dyn::Velocity&& velocity);

    //! \brief Default copy constructor
    NullSpaceMotion(const NullSpaceMotion&) = default;

    //! \brief Default move constructor
    NullSpaceMotion(NullSpaceMotion&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref NullSpaceMotion::external_velocity_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~NullSpaceMotion() = default;

    //! \brief Default copy operator
    NullSpaceMotion& operator=(const NullSpaceMotion&) = default;

    //! \brief Default move operator
    NullSpaceMotion& operator=(NullSpaceMotion&&) = default;

    //! \brief Read/write access the velocity used by the generator
    //! \return double& A reference to the velocity
    vector::dyn::Velocity& velocity();

    //! \brief Read access the velocity used by the generator
    //! \return double The velocity value
    const vector::dyn::Velocity& velocity() const;

    //! \brief Access to the shared pointer holding the velocity used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the forwarded
    //! velocity
    std::shared_ptr<vector::dyn::Velocity> velocityPtr() const;

protected:
    virtual void update(vector::dyn::Velocity& velocity) override;

    virtual void setRobot(Robot const* robot) override;

    std::shared_ptr<vector::dyn::Velocity> joint_velocity_;
    Eigen::MatrixXd null_space_projector_;
    Eigen::MatrixXd identity_;
};

} // namespace phri
