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
 * @file utilities.h
 * @author Benjamin Navarro
 * @brief Include all available utilities
 * @date April 2017
 * @ingroup OpenPHRI
 */

 #pragma once

#include <OpenPHRI/utilities/derivator.hpp>
#include <OpenPHRI/utilities/integrator.hpp>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <OpenPHRI/utilities/manipulator_equivalent_mass.h>
#include <OpenPHRI/utilities/interpolators.h>
#include <OpenPHRI/utilities/trajectory_generator.h>
#include <OpenPHRI/utilities/path_follower.h>
#include <OpenPHRI/utilities/laser_scanner_detector.h>
#include <OpenPHRI/utilities/data_logger.h>
#include <OpenPHRI/utilities/clock.h>
#include <OpenPHRI/utilities/benchmark.hpp>
#include <OpenPHRI/utilities/deadband.hpp>
