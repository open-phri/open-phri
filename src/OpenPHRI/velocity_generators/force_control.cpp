/*      File: force_control.cpp
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

#include <OpenPHRI/velocity_generators/force_control.h>
#include <iostream>

using namespace phri;

ForceControl::ForceControl(Vector6dConstPtr external_force_target,
                           double sample_time, Vector6dConstPtr p_gain,
                           Vector6dConstPtr d_gain, Vector6dConstPtr selection,
                           ReferenceFrame frame, ForceControlTargetType type)
    : VelocityGenerator(frame),
      external_force_target_(external_force_target),
      sample_time_(sample_time),
      p_gain_(p_gain),
      d_gain_(d_gain),
      selection_(selection),
      type_(type),
      filter_coeff_(1.) {
    prev_error_.setZero();
}

void ForceControl::configureFilter(double sample_time, double time_constant) {
    assert(sample_time > 0);
    assert(time_constant > 0);

    if (not(time_constant > 5. * sample_time)) {
        std::cout << "phri::ForceControl::configureFilter: the time constant ("
                  << time_constant
                  << ") for the low pass filter should be at least five times "
                     "greater than the sample time ("
                  << sample_time << ") to have a correct behavior" << std::endl;
    }

    filter_coeff_ = (sample_time / (time_constant + sample_time));
}

void ForceControl::update(Twist& velocity) {
    Vector6d error;
    Vector6d target;
    if (type_ == ForceControlTargetType::Environment) {
        target = *external_force_target_;
    } else {
        target = -*external_force_target_;
    }
    if (frame_ == ReferenceFrame::TCP) {
        error = target + static_cast<Vector6d>(robot_->task.state.wrench);
    } else {
        error = target + robot_->control.spatial_transformation_matrix *
                             static_cast<Vector6d>(robot_->task.state.wrench);
    }
    applySelection(error);

    error = filter_coeff_ * error + (1. - filter_coeff_) * prev_error_;

    velocity = p_gain_->cwiseProduct(error) +
               d_gain_->cwiseProduct((error - prev_error_) / sample_time_);

    prev_error_ = error;
}

void ForceControl::applySelection(Vector6d& vec) const {
    const auto& sel = *selection_;
    for (size_t i = 0; i < 6; ++i) {
        if (std::abs(sel(i)) < 1e-12) {
            vec(i) = 0.;
        }
    }
}
