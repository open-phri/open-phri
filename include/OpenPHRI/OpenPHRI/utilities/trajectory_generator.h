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
 * @file trajectory_generator.h
 * @author Benjamin Navarro
 * @brief Definition of the TrajectoryPoint struct and the TrajectoryGenerator class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/utilities/interpolators_common.h>
#include <OpenPHRI/utilities/object_collection.hpp>
#include <OpenPHRI/utilities/fifth_order_polynomial.h>
#include <vector>

namespace phri {

enum class TrajectoryOutputType {
	Position,
	Velocity,
	Acceleration
};

template<typename T>
class Trajectory {
public:
	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr<T> start, std::shared_ptr<T> output, double sample_time) :
		total_minimum_time_(0.),
		current_section_(0),
		sample_time_(sample_time),
		output_type_(output_type)
	{
		points_.push_back(start);
		output_ = output;
	}

	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr<T> start, T* output, double sample_time) :
		Trajectory(output_type, start, std::shared_ptr<T>(output, [](auto p){}), sample_time)
	{
	}

	Trajectory(TrajectoryOutputType output_type, TrajectoryPointPtr<T> start, double sample_time) :
		Trajectory(output_type, start, std::make_shared<T>(), sample_time)
	{
	}

	template<typename U = T>
	void addPathTo(TrajectoryPointPtr<T> to, T max_velocity, T max_acceleration, typename std::enable_if<std::is_arithmetic<U>::value>::type* = 0) {
		points_.push_back(to);
		PathParams<U> params;
		params.max_velocity = max_velocity;
		params.max_acceleration = max_acceleration;
		params.isFixedTime = false;
		path_params_.push_back(params);
	}

	template<typename U = T>
	void addPathTo(TrajectoryPointPtr<T> to, double duration, typename std::enable_if<std::is_arithmetic<U>::value>::type* = 0) {
		points_.push_back(to);
		PathParams<U> params;
		params.minimum_time = duration;
		params.isFixedTime = true;
		path_params_.push_back(params);
	}

	template<typename U = T>
	void addPathTo(TrajectoryPointPtr<T> to, T max_velocity, T max_acceleration, typename std::enable_if<not std::is_arithmetic<U>::value>::type* = 0) {
		points_.push_back(to);
		PathParams<U> params;
		params.max_velocity = max_velocity;
		params.max_acceleration = max_acceleration;
		params.isFixedTime = false;
		params.poly_params.resize(to->y->size());
		path_params_.push_back(params);
	}

	template<typename U = T>
	void addPathTo(TrajectoryPointPtr<T> to, double duration, typename std::enable_if<not std::is_arithmetic<U>::value>::type* = 0) {
		points_.push_back(to);
		PathParams<U> params;
		params.minimum_time = duration;
		params.isFixedTime = true;
		params.poly_params.resize(to->y->size());
		path_params_.push_back(params);
	}

	std::shared_ptr<const T> getOutput() {
		return output_;
	}

	double getCurrentPathMinimumTime() {
		return path_params_[current_section_].minimum_time;
	}

	double getTrajectoryMinimumTime() {
		return total_minimum_time_;
	}

	double getTrajectoryDuration() {
		double total = 0.;
		for (size_t i = 0; i < getSegmentCount(); ++i) {
			total += getPathDuration(i);
		}
		return total;
	}

	double getPathMinimumTime(size_t idx) {
		if(idx < points_.size()-1) {
			return path_params_[idx].minimum_time;
		}
		else {
			return 0.;
		}
	}

	double getPathDuration(size_t idx) {
		if(idx < points_.size()-1) {
			return path_params_[idx].minimum_time + path_params_[idx].padding_time;
		}
		else {
			return 0.;
		}
	}

	size_t getSegmentCount() {
		return points_.size()-1;
	}

	void setPaddingTime(size_t idx, double time) {
		if(idx < points_.size()-1) {
			path_params_[idx].padding_time = time;
		}
	}

	void setPathCurrentTime(size_t idx, double time) {
		if(idx < points_.size()-1) {
			path_params_[idx].current_time = time;
		}
	}

	template<typename U = T>
	bool computeParameters(typename std::enable_if<std::is_arithmetic<U>::value>::type* = 0) {
		int segments = points_.size()-1;
		if(segments < 1) {
			return false;
		}

		for (size_t i = 0; i < segments; ++i) {
			TrajectoryPoint<T>& from = *points_[i];
			TrajectoryPoint<T>& to = *points_[i+1];

			auto& params = path_params_[i];
			auto& poly_params = params.poly_params;

			poly_params = {0, params.minimum_time + params.padding_time, *from.y, *to.y, *from.dy, *to.dy, *from.d2y, *to.d2y};
			FifthOrderPolynomial::computeParameters(poly_params);

			params.current_time = 0.;
		}

		return true;
	}

	template<typename U = T>
	bool computeParameters(typename std::enable_if<not std::is_arithmetic<U>::value>::type* = 0) {
		int segments = points_.size()-1;
		if(segments < 1) {
			return false;
		}

		std::cout << "I'm here!" << std::endl;
		for (size_t i = 0; i < segments; ++i) {
			TrajectoryPoint<T>& from = *points_[i];
			TrajectoryPoint<T>& to = *points_[i+1];

			auto& params = path_params_[i];

			for (size_t j = 0; j < from.y->size(); ++j) {
				auto& poly_params = params.poly_params[j];
				poly_params = {0, params.minimum_time + params.padding_time, (*from.y)[j], (*to.y)[j], (*from.dy)[j], (*to.dy)[j], (*from.d2y)[j], (*to.d2y)[j]};
				FifthOrderPolynomial::computeParameters(poly_params);
			}

			params.current_time = 0.;
		}
		return true;
	}

	template<typename U = T>
	bool computeTimings(double v_eps = 1e-6, double a_eps = 1e-6, typename std::enable_if<std::is_arithmetic<U>::value>::type* = 0) {
		int segments = points_.size()-1;
		if(segments < 1) {
			return false;
		}

		bool ret = true;
		total_minimum_time_ = 0.;

		#pragma omp parallel for
		for (size_t i = 0; i < segments; ++i) {
			auto& params = path_params_[i];
			if(not params.isFixedTime) {
				TrajectoryPoint<T>& from = *points_[i];
				TrajectoryPoint<T>& to = *points_[i+1];
				bool ok = true;
				if(params.max_velocity < std::abs(*from.dy) or params.max_velocity < std::abs(*to.dy)) {
					std::cerr << "In computeTimings: initial or final velocity for segment " << i+1 << " is higher than the maximum" << std::endl;
					ok = false;
				}
				if(params.max_acceleration < std::abs(*from.d2y) or params.max_acceleration < std::abs(*to.d2y)) {
					std::cerr << "In computeTimings: initial or final acceleration for segment " << i+1 << " is higher than the maximum" << std::endl;
					ok = false;
				}
				if(not ok) {
					ret = false;
					continue;
				}

				auto& poly_params = params.poly_params;
				FifthOrderPolynomial::Parameters poly_params_v {0, 1., *from.y, *to.y, *from.dy, *to.dy, *from.d2y, *to.d2y};
				FifthOrderPolynomial::Parameters poly_params_a {0, 1., *from.y, *to.y, *from.dy, *to.dy, *from.d2y, *to.d2y};

				bool vmax_found = false;
				bool amax_found = false;
				double v_max, a_max;
				auto get_vmax_error = [&poly_params_v, &params, &v_max]() {
										  v_max = FifthOrderPolynomial::getFirstDerivativeMaximum(poly_params_v);
										  return v_max - params.max_velocity;
									  };
				auto get_amax_error = [&poly_params_a, &params, &a_max]() {
										  a_max = FifthOrderPolynomial::getSecondDerivativeMaximum(poly_params_a);
										  return a_max - params.max_acceleration;
									  };
				// std::cout << "--------------------------------------\n";
				extern size_t _compute_timings_total_iter;
				while(not (vmax_found and amax_found)) {

					if(not vmax_found) {
						FifthOrderPolynomial::computeParameters(poly_params_v);
						++_compute_timings_total_iter;
						double v_error_abs = std::abs(get_vmax_error());
						vmax_found = v_error_abs < v_eps;
						if(not vmax_found) {
							poly_params_v.xf = poly_params_v.xf*v_max/params.max_velocity;
						}
					}
					if(not amax_found) {
						FifthOrderPolynomial::computeParameters(poly_params_a);
						++_compute_timings_total_iter;
						double a_error_abs = std::abs(get_amax_error());
						amax_found = a_error_abs < a_eps;
						if(not amax_found) {
							poly_params_a.xf = poly_params_a.xf*(std::sqrt(a_max/params.max_acceleration));
						}
					}
					// std::cout << "vmax found: " << vmax_found << ", amax found: " << amax_found << std::endl;
				}

				if(poly_params_v.xf > poly_params_a.xf) {
					params.poly_params = poly_params_v;
				}
				else {
					params.poly_params = poly_params_a;
				}
				params.minimum_time = params.poly_params.xf;
			}

			total_minimum_time_ += params.minimum_time;
		}

		return ret;
	}

	template<typename U = T>
	bool computeTimings(double v_eps = 1e-6, double a_eps = 1e-6, typename std::enable_if<not std::is_arithmetic<U>::value>::type* = 0) {

		// TODO

		return true;
	}

	template<typename U = T>
	bool compute(typename std::enable_if<std::is_arithmetic<U>::value>::type* = 0) {
		if(current_section_ == points_.size()-1) {
			return true;
		}

		auto& params = path_params_[current_section_];

		double dt = (params.minimum_time + params.padding_time) - params.current_time;
		if(dt < 0.) {
			++current_section_;
			setPathCurrentTime(current_section_, -dt);
		}

		switch(output_type_) {
		case TrajectoryOutputType::Position:
			*output_ = FifthOrderPolynomial::compute(params.current_time, params.poly_params);
			break;
		case TrajectoryOutputType::Velocity:
			*output_ = FifthOrderPolynomial::computeFirstDerivative(params.current_time, params.poly_params);
			break;
		case TrajectoryOutputType::Acceleration:
			*output_ = FifthOrderPolynomial::computeSecondDerivative(params.current_time, params.poly_params);
			break;
		}

		params.current_time += sample_time_;

		return false;
	}

	template<typename U = T>
	bool compute(typename std::enable_if<not std::is_arithmetic<U>::value>::type* = 0) {

		// TODO

		return true;
	}

	double operator()() {
		return compute();
	}

	void reset() {
		current_section_ = 0;
		for(auto& param: path_params_) {
			param.current_time = 0.;
		}
	}

	static size_t getComputeTimingsIterations() {
		extern size_t _compute_timings_total_iter;
		return _compute_timings_total_iter;
	}

	static void resetComputeTimingsIterations() {
		extern size_t _compute_timings_total_iter;
		_compute_timings_total_iter = 0;
	}

private:
	template<typename U, typename Enable = void>
	struct PathParams {};

	template<typename U>
	struct PathParams<U, typename std::enable_if<std::is_arithmetic<U>::value>::type > {
		U max_velocity;
		U max_acceleration;
		bool isFixedTime;
		double minimum_time;
		double current_time;
		double padding_time;
		FifthOrderPolynomial::Parameters poly_params;
	};

	template<typename U>
	struct PathParams<U, typename std::enable_if<not std::is_arithmetic<U>::value>::type > {
		U max_velocity;
		U max_acceleration;
		bool isFixedTime;
		double minimum_time;
		double current_time;
		double padding_time;
		std::vector<FifthOrderPolynomial::Parameters> poly_params;
	};

	std::vector<TrajectoryPointPtr<T>> points_;
	std::vector<PathParams<T>> path_params_;
	double total_minimum_time_;
	size_t current_section_;
	double sample_time_;
	std::shared_ptr<T> output_;
	TrajectoryOutputType output_type_;
};

template<typename T>
using TrajectoryPtr = std::shared_ptr<Trajectory<T>>;
template<typename T>
using TrajectoryConstPtr = std::shared_ptr<const Trajectory<T>>;

extern template class Trajectory<double>;
extern template class Trajectory<VectorXd>;
extern template class Trajectory<Vector6d>;

enum class TrajectorySynchronization {
	NoSynchronization,
	SynchronizeWaypoints,
	SynchronizeTrajectory
};

/** @brief A trajectory generator with velocity and acceleration constraints.
 *  @details Can be used to generate multiple trajectories at once with or without synchronization (at waypoint or trajectory level).
 */
class TrajectoryGenerator : public ObjectCollection<TrajectoryPtr<double>> {
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
	/**
	 * @brief Call operator, shortcut for compute()
	 * @return The new output data.
	 */
	virtual double operator()() final;

	void reset();

private:
	TrajectorySynchronization sync_;
};

using TrajectoryGeneratorPtr = std::shared_ptr<TrajectoryGenerator>;
using TrajectoryGeneratorConstPtr = std::shared_ptr<const TrajectoryGenerator>;

} // namespace OpenPHRI
