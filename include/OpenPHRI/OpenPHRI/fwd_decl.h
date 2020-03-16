/*      File: fwd_decl.h
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
 * @file fwd_decl.h
 * @author Benjamin Navarro
 * @brief Some forward declaration to avoid the inclusion of unnecessary headers
 * in the interface
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

namespace phri {

class SafetyController;
class RobotModel;
class Driver;
class Constraint;
class SeparationDistanceConstraint;
class VelocityGenerator;
class JointVelocityGenerator;
class ForceGenerator;
class JointForceGenerator;
class LinearInterpolator;
class PolynomialInterpolator;

} // namespace phri

namespace YAML {
class Node;
}
