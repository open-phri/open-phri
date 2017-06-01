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
 * @file fwd_decl.h
 * @author Benjamin Navarro
 * @brief Some forward declaration to avoid the inclusion of unnecessary headers in the interface
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

namespace RSCL {

class SafetyController;
class Constraint;
class VelocityGenerator;
class JointVelocityGenerator;
class ForceGenerator;
class TorqueGenerator;
class LinearInterpolator;
class PolynomialInterpolator;

}
