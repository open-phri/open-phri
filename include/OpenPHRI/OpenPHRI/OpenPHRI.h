/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of OpenPHRI <https://gite.lirmm.fr/navarro/OpenPHRI>.
 *
 *  OpenPHRI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenPHRI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with OpenPHRI.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file OpenPHRI.h
 * @author Benjamin Navarro
 * @brief root include file for OpenPHRI library
 * @date April 2017
 */

/** @defgroup OpenPHRI
 * The OpenPHRI library provides a controller for safe human-robot interaction along with a set of constraints and velocity and force generators
 *
 * Usage: #include <OpenPHRI/OpenPHRI.h>
 *
 */

#pragma once

#include <OpenPHRI/definitions.h>
#include <OpenPHRI/robot.h>
#include <OpenPHRI/safety_controller.h>
#include <OpenPHRI/constraints.h>
#include <OpenPHRI/velocity_generators.h>
#include <OpenPHRI/joint_velocity_generators.h>
#include <OpenPHRI/force_generators.h>
#include <OpenPHRI/torque_generators.h>
#include <OpenPHRI/utilities.h>
