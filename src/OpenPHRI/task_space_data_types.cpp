/*      File: task_space_data_types.cpp
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
#include <OpenPHRI/task_space_data_types.h>

namespace phri {

/***			Pose			***/
Pose::Pose(ReferenceFrame frame)
    : Pose(phri::Vector3d::Zero(), Eigen::Quaterniond::Identity(), frame) {
}

Pose::Pose(phri::Vector3d translation, Eigen::Quaterniond orientation,
           ReferenceFrame frame)
    : TaskSpaceData<Pose>(frame),
      translation_(translation),
      orientation_(orientation) {
}

Pose::Pose(phri::Vector3d translation, phri::Vector3d euler_angles,
           ReferenceFrame frame)
    : Pose(translation,
           Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX()) // Roll
               * Eigen::AngleAxisd(euler_angles.y(),
                                   Eigen::Vector3d::UnitY()) // Pitch
               * Eigen::AngleAxisd(euler_angles.z(),
                                   Eigen::Vector3d::UnitZ()), // Yaw
           frame) {
}

Pose::Pose(phri::AffineTransform transformation, ReferenceFrame frame)
    : Pose(transformation.translation(),
           static_cast<Eigen::Quaterniond>(transformation.rotation()), frame) {
}

const phri::Vector3d& Pose::translation() const {
    return translation_;
}
const Eigen::Quaterniond& Pose::orientation() const {
    return orientation_;
}
phri::Vector3d& Pose::translation() {
    return translation_;
}
Eigen::Quaterniond& Pose::orientation() {
    return orientation_;
}

phri::Vector6d Pose::getErrorWith(const Pose& other) const {
    phri::Vector6d error;
    error.block<3, 1>(0, 0) = translation() - other.translation();
    error.block<3, 1>(3, 0) =
        orientation().getAngularError(other.orientation());
    return error;
}

Pose& Pose::integrate(const Twist& twist, double sample_time) {
    translation() += twist.translation() * sample_time;
    orientation().integrate(twist.rotation(), sample_time);
    return *this;
}

Pose::operator phri::AffineTransform() const {
    phri::AffineTransform transform;
    transform.setIdentity();
    transform.translate(translation());
    transform.rotate(orientation());
    return transform;
}

Pose::operator phri::Vector6d() const {
    phri::Vector6d dx;
    dx.block<3, 1>(0, 0) = translation();
    dx.block<3, 1>(3, 0) = orientation().getAngles();
    return dx;
}

Pose& Pose::operator=(const phri::Vector6d& pos_euler) {
    translation() = pos_euler.block<3, 1>(0, 0);
    orientation() =
        Eigen::AngleAxisd(pos_euler(3), Eigen::Vector3d::UnitX())    // Roll
        * Eigen::AngleAxisd(pos_euler(4), Eigen::Vector3d::UnitY())  // Pitch
        * Eigen::AngleAxisd(pos_euler(5), Eigen::Vector3d::UnitZ()); // Yaw
    return *this;
}

/***			Twist			***/
Twist::Twist(ReferenceFrame frame)
    : Twist(phri::Vector3d::Zero(), phri::Vector3d::Zero(), frame) {
}
Twist::Twist(const phri::Vector3d& translation, const phri::Vector3d& rotation,
             ReferenceFrame frame)
    : TaskSpaceData<Twist>(frame) {
    this->translation() = translation;
    this->rotation() = rotation;
}
Twist::Twist(const phri::Vector6d& twist, ReferenceFrame frame)
    : Twist(twist.segment<3>(0), twist.segment<3>(3), frame) {
}

Eigen::Ref<const phri::Vector3d> Twist::translation() const {
    return twist_.block<3, 1>(0, 0);
}
Eigen::Ref<const phri::Vector3d> Twist::rotation() const {
    return twist_.block<3, 1>(3, 0);
}
Eigen::Ref<phri::Vector3d> Twist::translation() {
    return twist_.block<3, 1>(0, 0);
}
Eigen::Ref<phri::Vector3d> Twist::rotation() {
    return twist_.block<3, 1>(3, 0);
}

Twist& Twist::integrate(const Acceleration& acceleration, double sample_time) {
    translation() += acceleration.translation() * sample_time;
    rotation() += acceleration.rotation() * sample_time;
    return *this;
}

Eigen::Ref<const phri::Vector6d> Twist::vector() const {
    return twist_;
}

Eigen::Ref<phri::Vector6d> Twist::vector() {
    return twist_;
}

Twist::operator const phri::Vector6d&() const {
    return twist_;
}

Twist::operator phri::Vector6d&() {
    return twist_;
}

const double* Twist::data() const {
    return twist_.data();
}

double* Twist::data() {
    return twist_.data();
}

Twist& Twist::operator=(const phri::Vector6d& twist) {
    translation() = twist.block<3, 1>(0, 0);
    rotation() = twist.block<3, 1>(3, 0);
    return *this;
}

Twist Twist::operator-(const phri::Twist& other) const {
    return Twist(*this) -= other;
}

Twist Twist::operator+(const phri::Twist& other) const {
    return Twist(*this) += other;
}

Twist Twist::operator*(double scalar) const {
    return Twist(*this) *= scalar;
}

Twist Twist::operator/(double scalar) const {
    return Twist(*this) /= scalar;
}

Twist& Twist::operator-=(const phri::Twist& other) {
    vector() -= other.transform(frame()).vector();
    return *this;
}

Twist& Twist::operator+=(const phri::Twist& other) {
    vector() += other.transform(frame()).vector();
    return *this;
}

Twist& Twist::operator*=(double scalar) {
    vector() *= scalar;
    return *this;
}

Twist& Twist::operator/=(double scalar) {
    vector() /= scalar;
    return *this;
}

Twist operator*(const AffineTransform& transform, const Twist& twist) {
    Twist new_twist;
    new_twist.translation() = transform.rotation() * twist.translation();
    new_twist.rotation() = transform.rotation() * twist.rotation();
    return new_twist;
}

/***			Acceleration			***/
Acceleration::Acceleration(ReferenceFrame frame)
    : Acceleration(phri::Vector3d::Zero(), phri::Vector3d::Zero(), frame) {
}
Acceleration::Acceleration(phri::Vector3d translation, phri::Vector3d rotation,
                           ReferenceFrame frame)
    : TaskSpaceData<Acceleration>(frame) {

    this->translation() = translation;
    this->rotation() = rotation;
}
Acceleration::Acceleration(phri::Vector6d acceleration, ReferenceFrame frame)
    : Acceleration(acceleration.segment<3>(0), acceleration.segment<3>(3),
                   frame) {
}

Eigen::Ref<const phri::Vector3d> Acceleration::translation() const {
    return acceleration_.block<3, 1>(0, 0);
}
Eigen::Ref<const phri::Vector3d> Acceleration::rotation() const {
    return acceleration_.block<3, 1>(3, 0);
}
Eigen::Ref<phri::Vector3d> Acceleration::translation() {
    return acceleration_.block<3, 1>(0, 0);
}
Eigen::Ref<phri::Vector3d> Acceleration::rotation() {
    return acceleration_.block<3, 1>(3, 0);
}

Eigen::Ref<const phri::Vector6d> Acceleration::vector() const {
    return acceleration_;
}

Eigen::Ref<phri::Vector6d> Acceleration::vector() {
    return acceleration_;
}

Acceleration::operator const phri::Vector6d&() const {
    return acceleration_;
}

Acceleration::operator phri::Vector6d&() {
    return acceleration_;
}

const double* Acceleration::data() const {
    return acceleration_.data();
}

double* Acceleration::data() {
    return acceleration_.data();
}

Acceleration& Acceleration::operator=(const phri::Vector6d& acceleration) {
    translation() = acceleration.block<3, 1>(0, 0);
    rotation() = acceleration.block<3, 1>(3, 0);
    return *this;
}

Acceleration Acceleration::operator-(const phri::Acceleration& other) const {
    return Acceleration(*this) -= other;
}

Acceleration Acceleration::operator+(const phri::Acceleration& other) const {
    return Acceleration(*this) += other;
}

Acceleration Acceleration::operator*(double scalar) const {
    return Acceleration(*this) *= scalar;
}

Acceleration Acceleration::operator/(double scalar) const {
    return Acceleration(*this) /= scalar;
}

Acceleration& Acceleration::operator-=(const phri::Acceleration& other) {
    vector() -= other.transform(frame()).vector();
    return *this;
}

Acceleration& Acceleration::operator+=(const phri::Acceleration& other) {
    vector() += other.transform(frame()).vector();
    return *this;
}

Acceleration& Acceleration::operator*=(double scalar) {
    vector() *= scalar;
    return *this;
}

Acceleration& Acceleration::operator/=(double scalar) {
    vector() /= scalar;
    return *this;
}

Acceleration operator*(const AffineTransform& transform,
                       const Acceleration& acceleration) {
    Acceleration new_acceleration;
    new_acceleration.translation() =
        transform.rotation() * acceleration.translation();
    new_acceleration.rotation() =
        transform.rotation() * acceleration.rotation();
    return new_acceleration;
}

/***			Wrench			***/
Wrench::Wrench(ReferenceFrame frame)
    : Wrench(phri::Vector3d::Zero(), phri::Vector3d::Zero(), frame) {
}
Wrench::Wrench(phri::Vector3d force, phri::Vector3d torque,
               ReferenceFrame frame)
    : TaskSpaceData<Wrench>(frame) {
    this->force() = force;
    this->torque() = torque;
}
Wrench::Wrench(phri::Vector6d wrench, ReferenceFrame frame)
    : Wrench(wrench.segment<3>(0), wrench.segment<3>(3), frame) {
}

Eigen::Ref<const phri::Vector3d> Wrench::force() const {
    return wrench_.block<3, 1>(0, 0);
}
Eigen::Ref<const phri::Vector3d> Wrench::torque() const {
    return wrench_.block<3, 1>(3, 0);
}
Eigen::Ref<phri::Vector3d> Wrench::force() {
    return wrench_.block<3, 1>(0, 0);
}
Eigen::Ref<phri::Vector3d> Wrench::torque() {
    return wrench_.block<3, 1>(3, 0);
}

Eigen::Ref<const phri::Vector6d> Wrench::vector() const {
    return wrench_;
}

Eigen::Ref<phri::Vector6d> Wrench::vector() {
    return wrench_;
}

Wrench::operator const phri::Vector6d&() const {
    return wrench_;
}

Wrench::operator phri::Vector6d&() {
    return wrench_;
}

const double* Wrench::data() const {
    return wrench_.data();
}

double* Wrench::data() {
    return wrench_.data();
}

Wrench& Wrench::operator=(const phri::Vector6d& wrench) {
    force() = wrench.block<3, 1>(0, 0);
    torque() = wrench.block<3, 1>(3, 0);
    return *this;
}

Wrench Wrench::operator-(const phri::Wrench& other) const {
    return Wrench(*this) -= other;
}

Wrench Wrench::operator+(const phri::Wrench& other) const {
    return Wrench(*this) += other;
}

Wrench Wrench::operator*(double scalar) const {
    return Wrench(*this) *= scalar;
}

Wrench Wrench::operator/(double scalar) const {
    return Wrench(*this) /= scalar;
}

Wrench& Wrench::operator-=(const phri::Wrench& other) {
    vector() -= other.transform(frame()).vector();
    return *this;
}

Wrench& Wrench::operator+=(const phri::Wrench& other) {
    vector() += other.transform(frame()).vector();
    return *this;
}

Wrench& Wrench::operator*=(double scalar) {
    vector() *= scalar;
    return *this;
}

Wrench& Wrench::operator/=(double scalar) {
    vector() /= scalar;
    return *this;
}

Wrench operator*(const AffineTransform& transform, const Wrench& wrench) {
    Wrench new_wrench;
    new_wrench.force() = transform.rotation() * wrench.force();
    new_wrench.torque() = transform.rotation() * wrench.torque();
    return new_wrench;
}

} // namespace phri
