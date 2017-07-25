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
 * @file utilities.h
 * @author Benjamin Navarro
 * @brief Include all available utilities
 * @date April 2017
 * @ingroup RSCL
 */

 #pragma once

#include <RSCL/utilities/derivator.hpp>
#include <RSCL/utilities/integrator.hpp>
#include <RSCL/utilities/object_collection.hpp>
#include <RSCL/utilities/manipulator_equivalent_mass.h>
#include <RSCL/utilities/interpolators.h>
#include <RSCL/utilities/trajectory_generator.h>
#include <RSCL/utilities/path_follower.h>
#include <RSCL/utilities/laser_scanner_detector.h>
#include <RSCL/utilities/data_logger.h>
#include <RSCL/utilities/clock.h>
#include <RSCL/utilities/benchmark.hpp>
#include <RSCL/utilities/deadband.hpp>
