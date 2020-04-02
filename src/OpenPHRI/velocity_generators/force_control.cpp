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
    const Eigen::Vector6d& proportional_gain,
    const Eigen::Vector6d& derivative_gain,
    const std::array<bool, 6>& selection_vector)
    : proportional_gain(proportional_gain),
      derivative_gain(derivative_gain),
      selection_vector(selection_vector) {
}

ForceControl::ForceControl(TargetType type)
    : ForceControl{spatial::Force::Zero(spatial::Frame::Ref(frame())),
                   Parameters{}, type} {
}

void ForceControl::configureFilter(scalar::TimeConstant time_constant) {
    auto time_constant_sec = time_constant.value();
    auto sample_time = robot().control().timeStep();
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

void ForceControl::configureFilter(scalar::CutoffFrequency cutoff_frequency) {
    configureFilter(cutoff_frequency.inverse());
}

void ForceControl::update(spatial::Velocity& velocity) {
    auto error = getTarget() - robot().task().state().force();

    // Zero the error on unused components
    applySelection(error);

    // Proportional action
    Eigen::Vector6d command =
        getParameters().proportional_gain.cwiseProduct(error);

    // Filter the error
    error = error * filter_coeff_ + prev_error_ * (1. - filter_coeff_);

    // Derivative action
    command += getParameters().derivative_gain.cwiseProduct(
        (error - prev_error_) / robot().control().timeStep());

    prev_error_ = error;

    velocity = spatial::Velocity(command, getTarget().frame());
}

void ForceControl::applySelection(Eigen::Vector6d& vec) const {
    const auto& sel = getParameters().selection_vector;
    for (size_t i = 0; i < 6; ++i) {
        if (not sel[i]) {
            vec(i) = 0.;
        }
    }
}

void ForceControl::setTarget(const spatial::Force& force) {
    target_.ref() = force;
}

const spatial::Force& ForceControl::getTarget() const {
    return target_.cref();
}

void ForceControl::setParameters(const Parameters& parameters) {
    parameters_.ref() = parameters;
}

const ForceControl::Parameters& ForceControl::getParameters() const {
    return parameters_.cref();
}

} // namespace phri