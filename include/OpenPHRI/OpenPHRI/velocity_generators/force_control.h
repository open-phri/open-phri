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
 * @file force_control.h
 * @author Benjamin Navarro
 * @brief Definition of the ForceControl class
 * @date May 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/definitions.h>

namespace OpenPHRI {

/** @brief Generates a velocity to regulate the external force to a given target using PD control.
 */
class ForceControl : public VelocityGenerator {
public:
	ForceControl(
		Vector6dConstPtr external_force_target,
		double sample_time,
		Vector6dConstPtr p_gain,
		Vector6dConstPtr d_gain,
		Vector6dConstPtr selection,
		ReferenceFrame frame = ReferenceFrame::TCP);

	~ForceControl() = default;

	virtual Vector6d compute() override;

	void configureFilter(double sample_time, double time_constant);

private:
	void applySelection(Vector6d& vec);

	Vector6dConstPtr external_force_target_;
	double sample_time_;
	Vector6dConstPtr p_gain_;
	Vector6dConstPtr d_gain_;
	Vector6dConstPtr selection_;
	ReferenceFrame frame_;

	double filter_coeff_;

	Vector6d prev_error_;
};

using ForceControlPtr = std::shared_ptr<ForceControl>;
using ForceControlConstPtr = std::shared_ptr<const ForceControl>;

} // namespace OpenPHRI
