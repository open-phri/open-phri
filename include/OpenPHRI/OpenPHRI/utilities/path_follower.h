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
 * @file path_follower.h
 * @author Benjamin Navarro
 * @brief Definition of the PathFollower class
 * @date April 2017
 * @ingroup OpenPHRI
 */


#pragma once

#include <OpenPHRI/utilities/trajectory_generator.h>
#include <unordered_map>

namespace OpenPHRI {

enum class PathStopAction {
	StopOne,
	StopAll
};

class PathFollower : public TrajectoryGenerator {
public:

	PathFollower(TrajectorySynchronization sync = TrajectorySynchronization::NoSynchronization, PathStopAction action = PathStopAction::StopOne);
	~PathFollower() = default;

	void setStopAction(PathStopAction action);

	virtual bool compute() override;

	virtual bool add(const std::string& name, TrajectoryPtr<double> item, doubleConstPtr current_value, doubleConstPtr target_value, doubleConstPtr max_error, bool force = false);
	virtual bool add(const std::string& name, TrajectoryPtr<double> item, const double* current_value, const double* target_value, double max_error, bool force = false);
	virtual bool remove(const std::string& name) override;

private:
	struct TrajectoryParameters {
		doubleConstPtr current_value;
		doubleConstPtr target_value;
		doubleConstPtr max_error;
	};

	using TrajectoryGenerator::add;
	PathStopAction action_;
	std::unordered_map<TrajectoryPtr<double>, TrajectoryParameters> traj_params_;
};

} // namespace OpenPHRI
