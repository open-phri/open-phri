/*      File: torque_proxy.h
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

/**
 * @file torque_proxy.h
 * @author Benjamin Navarro
 * @brief Definition of the TorqueProxy class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/torque_generators/torque_generator.h>
#include <OpenPHRI/definitions.h>

namespace phri {

/** @brief Generates a torque based on an externally managed one.
 *  @details Can be useful to add a velocity generated by an external library or
 * a torque sensor.
 */
class TorqueProxy : public TorqueGenerator {
public:
    /** @brief Construct a torque proxy given an externally managed torque
     */
    explicit TorqueProxy(VectorXdConstPtr torque);
    virtual ~TorqueProxy() = default;

protected:
    virtual void update(VectorXd& torque) override;

    VectorXdConstPtr torque_;
};

using TorqueProxyPtr = std::shared_ptr<TorqueProxy>;
using TorqueProxyConstPtr = std::shared_ptr<const TorqueProxy>;

} // namespace phri
