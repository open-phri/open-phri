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

namespace phri {

ForceControl::Parameters::Parameters() {
    proportional_gain.setZero();
    derivative_gain.setZero();
    selection_vector.fill(false);
}

ForceControl::Parameters::Parameters(
    const Vector6d& proportional_gain, const Vector6d& derivative_gain,
    const std::array<bool, 6>& selection_vector)
    : proportional_gain(proportional_gain),
      derivative_gain(derivative_gain),
      selection_vector(selection_vector) {
}

ForceControl::ForceControl(ReferenceFrame frame, TargetType type)
    : VelocityGenerator(frame),
      target_(std::make_shared<Wrench>()),
      parameters_(std::make_shared<Parameters>()),
      type_(type),
      filter_coeff_(1.) {
    prev_error_.setZero();
}

ForceControl::ForceControl(std::shared_ptr<Wrench> target,
                           std::shared_ptr<Parameters> parameters,
                           ReferenceFrame frame, TargetType type)
    : ForceControl(frame, type) {
    target_ = target;
    parameters_ = parameters;
}

ForceControl::ForceControl(Wrench& target, Parameters& parameters,
                           ReferenceFrame frame, TargetType type)
    : ForceControl(std::shared_ptr<Wrench>(&target, [](auto p) {}),
                   std::shared_ptr<Parameters>(&parameters, [](auto p) {}),
                   frame, type) {
}

ForceControl::ForceControl(const Wrench& target, const Parameters& parameters,
                           ReferenceFrame frame, TargetType type)
    : ForceControl(std::make_shared<Wrench>(target),
                   std::make_shared<Parameters>(parameters), frame, type) {
}

ForceControl::ForceControl(Wrench&& target, Parameters&& parameters,
                           ReferenceFrame frame, TargetType type)
    : ForceControl(std::make_shared<Wrench>(std::move(target)),
                   std::make_shared<Parameters>(std::move(parameters)), frame,
                   type) {
}

void ForceControl::configureFilter(units::time::second_t time_constant) {
    auto time_constant_sec = time_constant.to<double>();
    auto sample_time = robot_->control.time_step;
    assert(sample_time > 0);
    assert(time_constant_sec > 0);

    if (not(time_constant_sec > 5. * sample_time)) {
        std::cout << "phri::ForceControl::configureFilter: the time constant ("
                  << time_constant_sec
                  << ") for the low pass filter should be at least five times "
                     "greater than the sample time ("
                  << sample_time << ") to have a correct behavior" << std::endl;
    }

    filter_coeff_ = (sample_time / (time_constant_sec + sample_time));
}

void ForceControl::configureFilter(units::frequency::hertz_t cutoff_frequency) {
    configureFilter(units::time::second_t(
        1. / (2. * M_PI * cutoff_frequency.to<double>())));
}

void ForceControl::update(Twist& velocity) {
    Vector6d error;
    Vector6d force_target = target();
    if (type_ == TargetType::Robot) {
        force_target *= -1.;
    }
    if (frame_ == ReferenceFrame::TCP) {
        error = force_target + static_cast<Vector6d>(robot_->task.state.wrench);
    } else {
        error =
            force_target + robot_->control.spatial_transformation_matrix *
                               static_cast<Vector6d>(robot_->task.state.wrench);
    }
    applySelection(error);

    error = filter_coeff_ * error + (1. - filter_coeff_) * prev_error_;

    velocity = parameters().proportional_gain.cwiseProduct(error) +
               parameters().derivative_gain.cwiseProduct(
                   (error - prev_error_) / robot_->control.time_step);

    prev_error_ = error;
}

void ForceControl::applySelection(Vector6d& vec) const {
    const auto& sel = parameters().selection_vector;
    for (size_t i = 0; i < 6; ++i) {
        if (not sel[i]) {
            vec(i) = 0.;
        }
    }
}

Wrench& ForceControl::target() {
    return *target_;
}

Wrench ForceControl::target() const {
    return *target_;
}

std::shared_ptr<Wrench> ForceControl::targetPtr() const {
    return target_;
}

ForceControl::Parameters& ForceControl::parameters() {
    return *parameters_;
}

ForceControl::Parameters ForceControl::parameters() const {
    return *parameters_;
}

std::shared_ptr<ForceControl::Parameters> ForceControl::parametersPtr() const {
    return parameters_;
}

} // namespace phri