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
 * @file RSCL.h
 * @author Benjamin Navarro
 * @brief root include file for RSCL library
 * @date April 2014
 */

/** @defgroup RSCL
 * The RSCL library provides a controller for safe human-robot interaction along with a set of constraints and velocity and force generators
 *
 * Usage: #include <RSCL/RSCL.h>
 *
 */

#pragma once

#include <RSCL/definitions.h>
#include <RSCL/safety_controller.h>
#include <RSCL/constraints.h>
#include <RSCL/velocity_generators.h>
#include <RSCL/force_generators.h>
#include <RSCL/interpolators.h>
#include <RSCL/utilities.h>
