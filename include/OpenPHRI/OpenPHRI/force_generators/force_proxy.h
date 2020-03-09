/*      File: force_proxy.h
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

//! \file force_proxy.h
//! \author Benjamin Navarro
//! \brief Generates a force based on an externally managed one.
//! \date 05-2019
//! \ingroup phri

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

//! \brief Generates a force based on an externally managed one.
//! \details Can be useful to add a force generated by an external library or
//! a force sensor.
class ForceProxy : public ForceGenerator {
public:
    using generator = std::function<spatial::Force(void)>;

    //! \brief Construct a new ForceProxy object with an initial force set
    //! to zero.
    //! \details Use ForceProxy::force() to set it to the desired value
    // ForceProxy(spatial::Frame frame);
    ForceProxy() = default;

    //! \brief Construct a new ForceProxy object forwarding the given pointed
    //! value
    //! \param force The force to forward
    ForceProxy(std::shared_ptr<spatial::Force> force);

    //! \brief Construct a new ForceProxy object forwarding the given
    //! referenced value
    //! \param force The force to forward. Make sure that \p force
    //! outlives the generator
    ForceProxy(spatial::Force& force);

    //! \brief Construct a new ForceProxy object forwarding the given value
    //! \param force The force to forward
    ForceProxy(const spatial::Force& force);

    //! \brief Construct a new ForceProxy object forwarding the given value
    //! \param force The force to forward
    ForceProxy(spatial::Force&& force);

    //! \brief Construct a new ForceProxy object forwarding the force
    //! generated by \p generator
    //! \param force The force to forward
    ForceProxy(const generator& generator);

    //! \brief Default copy constructor
    ForceProxy(const ForceProxy&) = default;

    //! \brief Default move constructor
    ForceProxy(ForceProxy&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref ForceProxy::external_force_ was
    //! created using an rvalue reference, the pointed memory won't be released
    virtual ~ForceProxy() = default;

    //! \brief Default copy operator
    ForceProxy& operator=(const ForceProxy&) = default;

    //! \brief Default move operator
    ForceProxy& operator=(ForceProxy&&) = default;

    //! \brief Read/write access the force used by the generator
    //! \return double& A reference to the force
    spatial::Force& force();

    //! \brief Read access the force used by the generator
    //! \return double The force value
    const spatial::Force& force() const;

    //! \brief Access to the shared pointer holding the force used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the forwarded
    //! force
    std::shared_ptr<spatial::Force> forcePtr() const;

protected:
    virtual void update(spatial::Force& force) override;

    std::shared_ptr<spatial::Force> external_force_;
    generator generator_;
};

} // namespace phri
