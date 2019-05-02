/*      File: utilities.h
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
 * @file utilities.h
 * @author Benjamin Navarro
 * @brief Include all available utilities
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/utilities/benchmark.hpp>
#include <OpenPHRI/utilities/clock.h>
#include <OpenPHRI/utilities/data_logger.h>
#include <OpenPHRI/utilities/data_replayer.hpp>
#include <OpenPHRI/utilities/deadband.hpp>
#include <OpenPHRI/utilities/derivator.hpp>
#include <OpenPHRI/utilities/integrator.hpp>
#include <OpenPHRI/utilities/interpolators.h>
#include <OpenPHRI/utilities/laser_scanner_detector.h>
#include <OpenPHRI/utilities/manipulator_equivalent_mass.h>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <OpenPHRI/utilities/robot_model.h>
#include <OpenPHRI/utilities/task_space_trajectory_generator.h>
#include <OpenPHRI/utilities/trajectory_generator.h>
#include <OpenPHRI/utilities/low_pass_filter.hpp>

#include <OpenPHRI/utilities/app_maker.h>
