/*      File: joint_velocity_proxy.h
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

//! \file joint_velocity_proxy.h
//! \author Benjamin Navarro
//! \brief Generates a joint_velocity based on an externally managed one.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/joint_velocity_generators/joint_velocity_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

//! \brief Generates a joint velocity based on an externally managed one.
//! \details Can be useful to add a velocity generated by an external library or
//! to manually generate joint velocity profiles.
class JointVelocityProxy : public JointVelocityGenerator {
public:
    using generator = std::function<Eigen::VectorXd(void)>;

    //! \brief Construct a new JointVelocityProxy object with an initial
    //! velocity set to zero.
    //! \details Use JointVelocityProxy::velocity() to set it to the desired
    //! value
    JointVelocityProxy();

    //! \brief Construct a new JointVelocityProxy object forwarding the given
    //! pointed value
    //! \param velocity The velocity to forward
    explicit JointVelocityProxy(std::shared_ptr<Eigen::VectorXd> velocity);

    //! \brief Construct a new JointVelocityProxy object forwarding the given
    //! referenced value
    //! \param velocity The velocity to forward. Make sure that \p velocity
    //! outlives the generator
    explicit JointVelocityProxy(Eigen::VectorXd& velocity);

    //! \brief Construct a new JointVelocityProxy object forwarding the given
    //! value
    //! \param velocity The velocity to forward
    explicit JointVelocityProxy(const Eigen::VectorXd& velocity);

    //! \brief Construct a new JointVelocityProxy object forwarding the given
    //! value
    //! \param velocity The velocity to forward
    explicit JointVelocityProxy(Eigen::VectorXd&& velocity);

    //! \brief Construct a new JointVelocityProxy object forwarding the velocity
    //! generated by \p generator
    //! \param velocity The velocity to forward
    explicit JointVelocityProxy(const generator& generator);

    //! \brief Default copy constructor
    JointVelocityProxy(const JointVelocityProxy&) = default;

    //! \brief Default move constructor
    JointVelocityProxy(JointVelocityProxy&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref JointVelocityProxy::external_velocity_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~JointVelocityProxy() = default;

    //! \brief Default copy operator
    JointVelocityProxy& operator=(const JointVelocityProxy&) = default;

    //! \brief Default move operator
    JointVelocityProxy& operator=(JointVelocityProxy&&) = default;

    //! \brief Read/write access the velocity used by the generator
    //! \return double& A reference to the velocity
    Eigen::VectorXd& velocity();

    //! \brief Read access the velocity used by the generator
    //! \return double The velocity value
    Eigen::VectorXd velocity() const;

    //! \brief Access to the shared pointer holding the velocity used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the forwarded
    //! velocity
    std::shared_ptr<Eigen::VectorXd> velocityPtr() const;

protected:
    virtual void update(Eigen::VectorXd& velocity) override;

    virtual void setRobot(Robot const* robot) override;

    std::shared_ptr<Eigen::VectorXd> joint_velocity_;
    generator generator_;
};

} // namespace phri
