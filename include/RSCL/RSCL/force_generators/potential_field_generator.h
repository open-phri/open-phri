/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file potential_field_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the PotentialFieldGenerator class and related PotentialFieldType enum and PotentialFieldObject struct.
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/force_generators/force_generator.h>
#include <RSCL/utilities/object_collection.hpp>
#include <RSCL/definitions.h>
#include <map>

namespace RSCL {

/** @enum RSCL::PotentialFieldType
 *  @brief Defines if a PotentialFieldObject is repulsive or attractive.
 */
enum class PotentialFieldType {
	Attractive,
	Repulsive
};

/** @brief Hold the required property for any object of the potential field.
 */
struct PotentialFieldObject {
	PotentialFieldObject(
		PotentialFieldType type,
		doubleConstPtr gain,
		doubleConstPtr threshold_distance,
		Vector6dConstPtr object_position) :
		type(type),
		gain(gain),
		threshold_distance(threshold_distance),
		object_position(object_position)
	{

	}

	PotentialFieldType type;            /**< Type of object. See PotentialFieldType. */
	doubleConstPtr gain;                /**< Gain applied to get the resulting force. */
	doubleConstPtr threshold_distance;  /**< Distance at which the object's attractive or repulsive effect will start. */
	Vector6dConstPtr object_position;   /**< Object position in the chosen frame. */
};

using PotentialFieldObjectPtr = std::shared_ptr<PotentialFieldObject>;
using PotentialFieldObjectConstPtr = std::shared_ptr<const PotentialFieldObject>;

/** @brief A potential field generator for basic collision avoidance.
 *  @details Use a set of PotentialFieldObject to determine which force has to be applied to the TCP. "Based on Real-time obstacle avoidance for manipulators and mobile robots" by O. Khatib.
 */
class PotentialFieldGenerator : public ForceGenerator, public ObjectCollection<PotentialFieldObjectPtr> {
public:
	/**
	 * @brief Construct a potential field generator where objects position are given in the TCP frame.
	 */
	PotentialFieldGenerator();

	/**
	 * @brief Construct a potential field generator where objects position are given in the same frame as robot_positon.
	 * @param robot_position The positon of the robot in the same frame as the objects.
	 * @param spatial_transformation The spatial transformation matrix between the chosen frame and the TCP frame
	 * @param do_transpose [optional] If set to true, the spatial transformation matrix transpose will be used.
	 */
	PotentialFieldGenerator(
		Vector6dConstPtr robot_position,
		Matrix6dConstPtr spatial_transformation,
		bool do_transpose = true);
	~PotentialFieldGenerator() = default;

	virtual Vector6d compute() override;

private:
	Vector6dConstPtr robot_position_;
	Matrix6dConstPtr spatial_transformation_;
	bool do_transpose_;
};

using PotentialFieldGeneratorPtr = std::shared_ptr<PotentialFieldGenerator>;
using PotentialFieldGeneratorConstPtr = std::shared_ptr<const PotentialFieldGenerator>;

} // namespace RSCL
