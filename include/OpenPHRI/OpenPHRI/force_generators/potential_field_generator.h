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
#include <OpenPHRI/detail/universal_wrapper.hpp>

#include <physical_quantities/spatial/position.h>
#include <physical_quantities/spatial/force.h>

#include <map>

namespace phri {

/** @enum phri::PotentialFieldType
 *  @brief Defines if a PotentialFieldObject is repulsive or attractive.
 */
enum class PotentialFieldType { Attractive, Repulsive };

/** @brief Hold the required property for any object of the potential field.
 */
struct PotentialFieldObject {
    template <typename GainT, typename ThresT, typename PosT>
    PotentialFieldObject(PotentialFieldType type, GainT&& gain,
                         ThresT&& threshold_distance, PosT&& object_position)
        : type_(type),
          gain_(std::forward<GainT>(gain)),
          threshold_distance_(std::forward<ThresT>(threshold_distance)),
          object_position_(std::forward<PosT>(object_position)) {
    }

    [[nodiscard]] PotentialFieldType type() const;

    void setGain(const double& gain);
    [[nodiscard]] const double& getGain() const;

    void setThresholdDistance(const double& threshold);
    [[nodiscard]] const double& getThresholdDistance() const;

    void setObjectPosition(const spatial::Position& position);
    [[nodiscard]] const spatial::Position& getObjectPosition() const;

private:
    PotentialFieldType type_; /**< Type of object. See PotentialFieldType. */
    detail::UniversalWrapper<double>
        gain_; /**< Gain applied to get the resulting force. */
    detail::UniversalWrapper<double>
        threshold_distance_; /**< Distance at which the object's attractive or
                               repulsive effect will start. */
    detail::UniversalWrapper<spatial::Position>
        object_position_; /**< Object position in the chosen frame. */
};

/** @brief A potential field generator for basic collision avoidance.
 *  @details Use a set of PotentialFieldObject to determine which force has to
 * be applied to the TCP. "Based on Real-time obstacle avoidance for
 * manipulators and mobile robots" by O. Khatib.
 */
class PotentialFieldGenerator : public ForceGenerator,
                                public ObjectCollection<PotentialFieldObject> {
public:
    /**
     * @brief Construct a potential field generator where objects position are
     * given in the specified frame.
     */
    PotentialFieldGenerator();

    /**
     * @brief Construct a potential field generator where objects position are
     * given in the specified frame.
     * @param offset An offset in the TCP frame at which the distances will be
     * computed.
     */
    template <typename OffsetT>
    PotentialFieldGenerator(OffsetT&& offset)
        : offset_{std::forward<OffsetT>(offset)} {
    }

    virtual ~PotentialFieldGenerator() = default;

    void setOffset(const spatial::LinearPosition& offset);

    [[nodiscard]] const spatial::LinearPosition& getOffset() const;

protected:
    void update(spatial::Force& force) override;

    detail::UniversalWrapper<spatial::LinearPosition> offset_;
};

} // namespace phri
