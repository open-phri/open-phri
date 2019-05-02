/*      File: dummy_driver.h
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
 * @file dummy_driver.h
 * @author Benjamin Navarro
 * @brief Definition of the DummyDriver class
 * @date June 2018
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>
#include <OpenPHRI/drivers/driver.h>

namespace phri {

/** @brief A dummy driver that set its current joint state with the last command
 * received
 */
class DummyDriver : virtual public phri::Driver {
public:
    /**
     * @brief Construct a driver using a specific robot and a sample time.
     * @param robot The robot to read/write data from/to.
     * @param sample_time The sample time to use.
     */
    DummyDriver(phri::Robot& robot, double sample_time);

    /**
     * @brief Construct a driver using a specific robot and a YAML configuration
     * node.
     * @param robot The robot to read/write data from/to.
     * @param configuration The YAML configuration node.
     */
    DummyDriver(phri::Robot& robot, const YAML::Node& configuration);

    virtual ~DummyDriver();

    virtual bool start(double timeout = 0.) override;
    virtual bool stop() override;

    virtual bool sync() override;
    virtual bool read() override;
    virtual bool send() override;

private:
    static bool registered_in_factory;
};

using DummyDriverPtr = std::shared_ptr<DummyDriver>;
using DummyDriverConstPtr = std::shared_ptr<const DummyDriver>;

} // namespace phri
