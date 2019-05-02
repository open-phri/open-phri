/*      File: force_constraint.h
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
 * @file force_constraint.h
 * @author Benjamin Navarro
 * @brief Definition of the ForceConstraint class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/constraints/velocity_constraint.h>
#include <OpenPHRI/velocity_generators/velocity_generator.h>

namespace phri {

/** @brief A constraint to limit the external force.
 *  @details MUST NOT BE USED FOR NOW
 */
class ForceConstraint : public Constraint {
public:
    /***		Constructor & destructor		***/

    /**
     * @brief Construct a force constraint with a given external and maximum
     * force.
     * @param maximum_force A shared pointer to the maximum external force
     * allowed.
     */
    ForceConstraint(VelocityConstraintPtr constraint,
                    doubleConstPtr maximum_force);

    virtual ~ForceConstraint() = default;

    VelocityGeneratorPtr getVelocityGenerator() const;

    /***		Algorithm		***/
    virtual double compute() override;

private:
    VelocityConstraintPtr constraint_;
    VelocityGeneratorPtr velocity_generator_;
    doubleConstPtr maximum_force_;
};

using ForceConstraintPtr = std::shared_ptr<ForceConstraint>;
using ForceConstraintConstPtr = std::shared_ptr<const ForceConstraint>;

} // namespace phri
