/*      File: constraints.h
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
 * @file constraints.h
 * @author Benjamin Navarro
 * @brief Include all implemented constraints
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/constraints/default_constraint.h>
#include <OpenPHRI/constraints/velocity_constraint.h>
#include <OpenPHRI/constraints/acceleration_constraint.h>
#include <OpenPHRI/constraints/power_constraint.h>
#include <OpenPHRI/constraints/task_emergency_stop_constraint.h>
#include <OpenPHRI/constraints/joint_emergency_stop_constraint.h>
#include <OpenPHRI/constraints/separation_distance_constraint.h>
#include <OpenPHRI/constraints/joint_velocity_constraint.h>
#include <OpenPHRI/constraints/joint_acceleration_constraint.h>
#include <OpenPHRI/constraints/kinetic_energy_constraint.h>
