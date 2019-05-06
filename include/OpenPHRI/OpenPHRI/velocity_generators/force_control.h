/*      File: force_control.h
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

//! \file force_control.h
//! \author Benjamin Navarro
//! \brief Generates a velocity to regulate the external force to a given target
//! using PD control \date 05-2019 \ingroup phri

#pragma once

#include <OpenPHRI/velocity_generators/velocity_generator.h>
#include <OpenPHRI/definitions.h>
#include <array>

namespace phri {

//! \brief Generates a velocity to regulate the external force to a given target
//! using PD control
class ForceControl : public VelocityGenerator {
public:
    //! \brief Specify for force target type
    enum class TargetType {
        //! \brief The target force is the one applied to the environment
        Environment,
        //! \brief The target force is the one applied to the robot
        Robot
    };

    //! \brief Contains the control parameters for the ForceControl generator
    struct Parameters {
        //! \brief Construct a new Force Control Parameters object with gains
        //! set to zero and no degrees of freedom selected
        Parameters();

        //! \brief Construct a new Force Control Parameters object with the
        //! given gains and selection vector \param proportional_gain Gain
        //! applied to the force error \param derivative_gain Gain applied to
        //! the force error derivative \param selection_vector A non-zero value
        //! indicates that equivalent degree of freedom is force controled
        Parameters(const Vector6d& proportional_gain,
                   const Vector6d& derivative_gain,
                   const std::array<bool, 6>& selection_vector);

        //! \brief Gain applied to the force error
        Vector6d proportional_gain;
        //! \brief Gain applied to the force error derivative
        Vector6d derivative_gain;
        //! \brief A non-zero value indicates that equivalent degree of freedom
        //! is force controled
        std::array<bool, 6> selection_vector;
    };

    //! \brief  Construct a new Force Control object with default parameters and
    //! with an initial target set to zero
    //! \details Use ForceControl::target() to set the desired force value
    //! \param frame The reference frame in which the target force is expressed.
    //! \param type If the regulated force is the one applied to the environment
    //! or to the robot
    ForceControl(ReferenceFrame frame,
                 TargetType type = TargetType::Environment);

    //! \brief  Construct a new Force Control object with given pointed target
    //! and parameters
    //! \details Use ForceControl::target() to set the desired force value
    //! \param target The targeted force
    //! \param parameters The force control law parameters
    //! \param frame The reference frame in which the target force is expressed.
    //! \param type If the regulated force is the one applied to the environment
    //! or to the robot
    ForceControl(std::shared_ptr<Wrench> target,
                 std::shared_ptr<Parameters> parameters, ReferenceFrame frame,
                 TargetType type = TargetType::Environment);

    //! \brief  Construct a new Force Control object with given referenced
    //! target and parameters
    //! \details Use ForceControl::target() to set the desired force value
    //! \param target The targeted force. Make sure that \p target outlives the
    //! generator
    //! \param parameters The force control law parameters. Make sure that \p
    //! parameters outlives the generator
    //! \param frame The reference frame in which the target force is expressed.
    //! \param type If the regulated force is the one applied to the environment
    //! or to the robot
    ForceControl(Wrench& target, Parameters& parameters, ReferenceFrame frame,
                 TargetType type = TargetType::Environment);

    //! \brief  Construct a new Force Control object with given target
    //! and parameters values
    //! \details Use ForceControl::target() to set the desired force value
    //! \param target The targeted force
    //! \param parameters The force control law parameters
    //! \param frame The reference frame in which the target force is expressed.
    //! \param type If the regulated force is the one applied to the environment
    //! or to the robot
    ForceControl(const Wrench& target, const Parameters& parameters,
                 ReferenceFrame frame,
                 TargetType type = TargetType::Environment);

    //! \brief  Construct a new Force Control object with given target
    //! and parameters values
    //! \details Use ForceControl::target() to set the desired force value
    //! \param target The targeted force
    //! \param parameters The force control law parameters
    //! \param frame The reference frame in which the target force is expressed.
    //! \param type If the regulated force is the one applied to the environment
    //! or to the robot
    ForceControl(Wrench&& target, Parameters&& parameters, ReferenceFrame frame,
                 TargetType type = TargetType::Environment);

    //! \brief Default copy constructor
    ForceControl(const ForceControl&) = default;

    //! \brief Default move constructor
    ForceControl(ForceControl&&) = default;

    //! \brief Default virtual destructor
    //! \details If \ref ForceControl::target_ and ForceControl::parameters_
    //! were created using rvalue references, the pointed memories won't be
    //! released
    virtual ~ForceControl() = default;

    //! \brief Default copy operator
    ForceControl& operator=(const ForceControl&) = default;

    //! \brief Default move operator
    ForceControl& operator=(ForceControl&&) = default;

    //! \brief Configure the filter acting on the error
    //! \details Using this filter will provide a smoother derivative of the
    //! error at the cost of a delay. A simple first order low-pass filter is
    //! used internally.
    //! \param time_constant The time constant for the filter
    void configureFilter(units::time::second_t time_constant);

    //! \brief Configure the filter acting on the error
    //! \details Using this filter will provide a smoother derivative of the
    //! error at the cost of a delay. A simple first order low-pass filter is
    //! used internally.
    //! \param cutoff_frequency The filter cutoff frequency
    void configureFilter(units::frequency::hertz_t cutoff_frequency);

    //! \brief Read/write access the target wrench used by the generator
    //! \return double& A reference to the target wrench
    Wrench& target();

    //! \brief Read access the target wrench used by the generator
    //! \return double The target wrench value
    Wrench target() const;

    //! \brief Access to the shared pointer holding the target wrench used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the wrench target
    std::shared_ptr<Wrench> targetPtr() const;

    //! \brief Read/write access the control parameters used by the generator
    //! \return double& A reference to the control parameters
    Parameters& parameters();

    //! \brief Read access the control parameters used by the generator
    //! \return double The target control parameters
    Parameters parameters() const;

    //! \brief Access to the shared pointer holding the control parameters used
    //! by the generator
    //! \return std::shared_ptr<double> A shared pointer to the control
    //! parameters
    std::shared_ptr<Parameters> parametersPtr() const;

protected:
    virtual void update(Twist& velocity) override;
    void applySelection(Vector6d& vec) const;

    std::shared_ptr<Wrench> target_;
    std::shared_ptr<Parameters> parameters_;
    TargetType type_;
    double filter_coeff_;
    Vector6d prev_error_;
};

} // namespace phri
