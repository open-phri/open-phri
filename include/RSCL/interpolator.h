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
 * @file interpolator.h
 * @author Benjamin Navarro
 * @brief Base class definition for interpolators
 * @date April 2014
 * @ingroup RSCL
 */

#pragma once

#include <RSCL/definitions.h>

namespace RSCL {

/** @brief Base class for all interpolators.
 *  @details Provides two pure virtual method to update the internal parameters and to compute a new value.
 */
class Interpolator {
public:
	Interpolator() = default;
	~Interpolator() = default;

	/**
	 * @brief Get the pointer to the output data.
	 * @return A shared pointer to the output data.
	 */
	virtual doubleConstPtr getOutput() const final;

	/**
	 * @brief Set the pointer to the input data.
	 * @return A shared pointer to the input data.
	 */
	virtual void setInput(doubleConstPtr input) final;

	/**
	 * @brief Compute the interpolator's parameters .
	 */
	virtual void computeParameters() = 0;

	/**
	 * @brief Compute the new interpolator output.
	 * @return The new output data.
	 */
	virtual double compute() = 0;

protected:
	doubleConstPtr input_;
	doublePtr output_;
};

using InterpolatorPtr = std::shared_ptr<Interpolator>;
using InterpolatorConstPtr = std::shared_ptr<const Interpolator>;

} // namespace RSCL
