/*      File: task_space_data_types.h
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
/**
 * @file task_space_data_types.h
 * @author Benjamin Navarro
 * @brief Usefull definitions being used in OpenPHRI
 * @date September 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <OpenPHRI/type_aliases.h>

namespace phri {

class Twist;
class Acceleration;

class Pose {
public:
    Pose();
    Pose(phri::Vector3d translation, Eigen::Quaterniond orientation);
    Pose(phri::Vector3d translation, phri::Vector3d euler_angles);
    explicit Pose(phri::AffineTransform transformation);

    const phri::Vector3d& translation() const;
    const Eigen::Quaterniond& orientation() const;
    phri::Vector3d& translation();
    Eigen::Quaterniond& orientation();

    phri::Vector6d getErrorWith(const Pose& other) const;
    Pose& integrate(const Twist& twist, double sample_time);

    operator phri::AffineTransform() const;
    explicit operator phri::Vector6d() const;
    Pose& operator=(const phri::Vector6d& pos_euler);

    friend std ::ostream& operator<<(std::ostream& out, const Pose& pose) {
        out << pose.translation().x() << "x + " << pose.translation().y()
            << "y + " << pose.translation().z() << "z; ";
        out << pose.orientation().w() << " + " << pose.orientation().x()
            << "i + " << pose.orientation().y() << "j + "
            << pose.orientation().z() << "k";
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector3d translation_;
    Eigen::Quaterniond orientation_;
};

using PosePtr = std::shared_ptr<Pose>;
using PoseConstPtr = std::shared_ptr<const Pose>;

class Twist {
public:
    Twist();
    Twist(const phri::Vector3d& translation, const phri::Vector3d& rotation);
    explicit Twist(const phri::Vector6d& twist);

    Eigen::Ref<const phri::Vector3d> translation() const;
    Eigen::Ref<const phri::Vector3d> rotation() const;
    Eigen::Ref<phri::Vector3d> translation();
    Eigen::Ref<phri::Vector3d> rotation();

    Twist& integrate(const Acceleration& acceleration, double sample_time);

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Twist& operator=(const phri::Vector6d& twist);
    Twist operator-(const phri::Twist& other) const;
    Twist operator+(const phri::Twist& other) const;
    Twist operator*(double scalar) const;
    Twist operator/(double scalar) const;

    friend std ::ostream& operator<<(std::ostream& out, const Twist& twist) {
        out << static_cast<phri::Vector6d>(twist).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d twist_;
};

using TwistPtr = std::shared_ptr<Twist>;
using TwistConstPtr = std::shared_ptr<const Twist>;

class Acceleration {
public:
    Acceleration();
    Acceleration(phri::Vector3d translation, phri::Vector3d rotation);
    explicit Acceleration(phri::Vector6d acceleration);

    Eigen::Ref<const phri::Vector3d> translation() const;
    Eigen::Ref<const phri::Vector3d> rotation() const;
    Eigen::Ref<phri::Vector3d> translation();
    Eigen::Ref<phri::Vector3d> rotation();

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Acceleration& operator=(const phri::Vector6d& acceleration);

    friend std ::ostream& operator<<(std::ostream& out,
                                     const Acceleration& acceleration) {
        out << static_cast<phri::Vector6d>(acceleration).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d acceleration_;
};

using AccelerationPtr = std::shared_ptr<Acceleration>;
using AccelerationConstPtr = std::shared_ptr<const Acceleration>;

class Wrench {
public:
    Wrench();
    Wrench(phri::Vector3d force, phri::Vector3d torque);
    explicit Wrench(phri::Vector6d wrench);

    Eigen::Ref<const phri::Vector3d> force() const;
    Eigen::Ref<const phri::Vector3d> torque() const;
    Eigen::Ref<phri::Vector3d> force();
    Eigen::Ref<phri::Vector3d> torque();

    Eigen::Ref<const phri::Vector6d> vector() const;
    Eigen::Ref<phri::Vector6d> vector();

    operator const phri::Vector6d&() const;
    operator phri::Vector6d&();

    double* data();
    const double* data() const;

    Wrench& operator=(const phri::Vector6d& wrench);

    friend std ::ostream& operator<<(std::ostream& out, const Wrench& wrench) {
        out << static_cast<phri::Vector6d>(wrench).transpose();
        return (out);
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
private:
    phri::Vector6d wrench_;
};

using WrenchPtr = std::shared_ptr<Wrench>;
using WrenchConstPtr = std::shared_ptr<const Wrench>;

} // namespace phri
