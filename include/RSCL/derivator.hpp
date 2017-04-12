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
 * @file derivator.hpp
 * @author Benjamin Navarro
 * @brief Generir first order derivator
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

/** @brief A generic derivator class.
 *  @details Works on primitive types and Eigen vectors
 *  @tparam T The type to work on
 */
template<typename T>
class Derivator {
public:
	/***		Constructor & destructor		***/

	/**
	 * @brief Construct a Derivator for a given input and sample time.
	 * @param input A shared pointer to the input data
	 * @param sample_time Time step between each call to Derivator::compute().
	 */
	Derivator(std::shared_ptr<const T> input, double sample_time) :
		input_(input), frequency_(1./sample_time)
	{
		output_ = std::make_shared<T>();

		reset();
	}

	virtual ~Derivator() = default;

	/***		Configuration		***/

	/**
	 * @brief Get the pointer to the output data
	 * @return A shared pointer to the output data.
	 */
	std::shared_ptr<const T> getOutput() {
		return output_;
	}

	/**
	 * @brief Reset the derivator's internal state to its original state
	 */
	template<typename TT = T>
	typename std::enable_if<std::is_same<TT, Eigen::Matrix<typename TT::Scalar,
	                                                       TT::RowsAtCompileTime, TT::ColsAtCompileTime,
	                                                       TT::Options, TT::MaxRowsAtCompileTime,
	                                                       TT::MaxColsAtCompileTime>>::value, void>::type
	reset() {
		output_->setZero();
		previous_input_ = *input_;
	}

	/**
	 * @brief Reset the derivator's internal state to its original state
	 */
	template<typename TT = T>
	typename std::enable_if<std::is_arithmetic<TT>::value, void>::type
	reset() {
		*output_ = TT(0);
		previous_input_ = *input_;
	}

	/***		Algorithm		***/

	/**
	 * @brief Update the derivative estimation.
	 * @return The new estimation.
	 */
	virtual T compute() {
		*output_ = (*input_ - previous_input_) * frequency_;
		previous_input_ = *input_;
		return *output_;
	}

private:
	std::shared_ptr<const T> input_;
	std::shared_ptr<T> output_;
	T previous_input_;
	double frequency_;
};

template<typename T>
using DerivatorPtr = std::shared_ptr<Derivator<T>>;
template<typename T>
using DerivatorConstPtr = std::shared_ptr<const Derivator<T>>;

extern template class Derivator<double>;
extern template class Derivator<Vector2d>;
extern template class Derivator<Vector3d>;
extern template class Derivator<Vector6d>;

} // namespace RSCL
