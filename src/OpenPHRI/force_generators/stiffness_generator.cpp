/*      File: stiffness_generator.cpp
*       This file is part of the program open-phri
*       Program description : OpenPHRI: a generic framework to easily and safely control robots in interactions with humans
*       Copyright (C) 2017 -  Benjamin Navarro (LIRMM). All Right reserved.
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
*       You should have received a copy of the GNU Lesser General Public License version 3 and the
*       General Public License version 3 along with this program.
*       If not, see <http://www.gnu.org/licenses/>.
*/

#include <OpenPHRI/force_generators/stiffness_generator.h>
#include <iostream>

using namespace phri;

StiffnessGenerator::StiffnessGenerator(
	Matrix6dConstPtr stiffness,
	Vector6dConstPtr target_position,
	ReferenceFrame stiffness_frame,
	ReferenceFrame target_position_frame) :
	ForceGenerator(stiffness_frame),
	stiffness_(stiffness),
	target_position_(target_position),
	stiffness_frame_(stiffness_frame),
	target_position_frame_(target_position_frame)
{
}


Vector6d StiffnessGenerator::compute() {
	Vector6d error;

	if(stiffness_frame_ == ReferenceFrame::TCP) {
		if(target_position_frame_ == ReferenceFrame::TCP) {
			error = *target_position_;
		}
		else {
			error = robot_->spatialTransformationMatrix()->transpose() * (*target_position_ - *robot_->controlPointCurrentPose());
		}
	}
	else {
		if(target_position_frame_ == ReferenceFrame::TCP) {
			error = *robot_->spatialTransformationMatrix() * *target_position_;
		}
		else {
			error = *target_position_ - *robot_->controlPointCurrentPose();
		}
	}

	force_ = *stiffness_ * error;

	return ForceGenerator::compute();
}
