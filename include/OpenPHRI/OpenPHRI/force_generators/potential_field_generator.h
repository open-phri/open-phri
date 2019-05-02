/*      File: potential_field_generator.h
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
 * @file potential_field_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the PotentialFieldGenerator class and related
 * PotentialFieldType enum and PotentialFieldObject struct.
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/force_generators/force_generator.h>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <OpenPHRI/definitions.h>
#include <map>

namespace phri {

/** @enum phri::PotentialFieldType
 *  @brief Defines if a PotentialFieldObject is repulsive or attractive.
 */
enum class PotentialFieldType { Attractive, Repulsive };

/** @brief Hold the required property for any object of the potential field.
 */
struct PotentialFieldObject {
    PotentialFieldObject(PotentialFieldType type, doubleConstPtr gain,
                         doubleConstPtr threshold_distance,
                         PoseConstPtr object_position)
        : type(type),
          gain(gain),
          threshold_distance(threshold_distance),
          object_position(object_position) {
    }

    PotentialFieldType type; /**< Type of object. See PotentialFieldType. */
    doubleConstPtr gain;     /**< Gain applied to get the resulting force. */
    doubleConstPtr
        threshold_distance; /**< Distance at which the object's attractive or
                               repulsive effect will start. */
    PoseConstPtr object_position; /**< Object position in the chosen frame. */
};

using PotentialFieldObjectPtr = std::shared_ptr<PotentialFieldObject>;
using PotentialFieldObjectConstPtr =
    std::shared_ptr<const PotentialFieldObject>;

/** @brief A potential field generator for basic collision avoidance.
 *  @details Use a set of PotentialFieldObject to determine which force has to
 * be applied to the TCP. "Based on Real-time obstacle avoidance for
 * manipulators and mobile robots" by O. Khatib.
 */
class PotentialFieldGenerator
    : public ForceGenerator,
      public ObjectCollection<PotentialFieldObjectPtr> {
public:
    /**
     * @brief Construct a potential field generator where objects position are
     * given in the specified frame.
     * @param objects_frame The frame in which the positons of the objects are
     * expressed .
     */
    explicit PotentialFieldGenerator(
        ReferenceFrame objects_frame = ReferenceFrame::TCP);

    /**
     * @brief Construct a potential field generator where objects position are
     * given in the specified frame.
     * @param offset An offset in the TCP frame at which the distances will be
     * computed.
     * @param objects_frame The frame in which the positons of the objects are
     * expressed.
     */
    PotentialFieldGenerator(Vector3dConstPtr offset,
                            ReferenceFrame objects_frame = ReferenceFrame::TCP);
    virtual ~PotentialFieldGenerator() = default;

protected:
    virtual void update(Vector6d& force) override;

    ReferenceFrame objects_frame_;
    Vector3dConstPtr offset_;
};

using PotentialFieldGeneratorPtr = std::shared_ptr<PotentialFieldGenerator>;
using PotentialFieldGeneratorConstPtr =
    std::shared_ptr<const PotentialFieldGenerator>;

} // namespace phri
