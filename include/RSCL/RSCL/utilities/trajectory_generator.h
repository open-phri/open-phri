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
 * @file trajectory_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the TrajectoryPoint struct and the TrajectoryGenerator class
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/utilities/interpolators_common.h>
#include <RSCL/utilities/object_collection.hpp>
#include <RSCL/utilities/fifth_order_polynomial.h>
#include <vector>

namespace RSCL {

enum class TrajectoryOutputType {
	Position,
	Velocity,
	Acceleration
};

class Trajectory {
public:

	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr start, doublePtr output, double sample_time);
	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr start, double* output, double sample_time);
	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr start, double sample_time);

	void addPathTo(TrajectoryPointPtr to, double max_velocity, double max_acceleration);
	void addPathTo(TrajectoryPointPtr to, double duration);

	doubleConstPtr getOutput();

	bool computeParameters();
	bool computeTimings(double v_eps = 1e-6, double a_eps = 1e-6);
	bool compute();

	double getCurrentPathMinimumTime();
	double getTrajectoryMinimumTime();
	double getTrajectoryDuration();
	double getPathMinimumTime(size_t idx);
	double getPathDuration(size_t idx);
	size_t getSegmentCount();

	void setPaddingTime(size_t idx, double time);
	void setPathCurrentTime(size_t idx, double time);

private:
	struct PathParams {
		double max_velocity;
		double max_acceleration;
		bool isFixedTime;
		double minimum_time;
		double current_time;
		double padding_time;
		FifthOrderPolynomial::Parameters poly_params;
	};

	std::vector<TrajectoryPointPtr> points_;
	std::vector<PathParams> path_params_;
	double total_minimum_time_;
	size_t current_section_;
	double sample_time_;
	doublePtr output_;
	TrajectoryOutputType output_type_;
};

using TrajectoryPtr = std::shared_ptr<Trajectory>;
using TrajectoryConstPtr = std::shared_ptr<const Trajectory>;

enum class TrajectorySynchronization {
	NoSynchronization,
	SynchronizeWaypoints,
	SynchronizeTrajectory
};

/** @brief A trajectory generator with velocity and acceleration constraints.
 *  @details Can be used to generate multiple trajectories at once with or without synchronization (at waypoint or trajectory level).
 */
class TrajectoryGenerator : public ObjectCollection<TrajectoryPtr> {
public:

	/**
	 * @brief Construct a polynomial interpolator given starting and ending points and a user defined input
	 * @param from Starting point.
	 * @param to Ending point.
	 * @param input A shared pointer to the input value used by the interpolator.
	 */
	TrajectoryGenerator(TrajectorySynchronization sync = TrajectorySynchronization::NoSynchronization);

	~TrajectoryGenerator() = default;

	void setSynchronizationMethod(TrajectorySynchronization sync);

	virtual void computeParameters(double v_eps = 1e-6, double a_eps = 1e-6);
	virtual bool compute();

private:
	TrajectorySynchronization sync_;
};

using TrajectoryGeneratorPtr = std::shared_ptr<TrajectoryGenerator>;
using TrajectoryGeneratorConstPtr = std::shared_ptr<const TrajectoryGenerator>;

} // namespace RSCL
