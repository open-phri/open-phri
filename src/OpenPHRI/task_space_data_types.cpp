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
Pose::Pose() : Pose(phri::Vector3d::Zero(), Eigen::Quaterniond::Identity()) {
}

Pose::Pose(phri::Vector3d translation, Eigen::Quaterniond orientation)
    : translation_(translation), orientation_(orientation) {
}

Pose::Pose(phri::Vector3d translation, phri::Vector3d euler_angles)
    : Pose(translation,
           Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX()) // Roll
               * Eigen::AngleAxisd(euler_angles.y(),
                                   Eigen::Vector3d::UnitY()) // Pitch
               * Eigen::AngleAxisd(euler_angles.z(),
                                   Eigen::Vector3d::UnitZ())) // Yaw
{
}

Pose::Pose(phri::AffineTransform transformation)
    : Pose(transformation.translation(),
           static_cast<Eigen::Quaterniond>(transformation.rotation())) {
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
Twist::Twist() : Twist(phri::Vector3d::Zero(), phri::Vector3d::Zero()) {
}
Twist::Twist(phri::Vector3d translation, phri::Vector3d rotation) {
    twist_.block<3, 1>(0, 0) = translation;
    twist_.block<3, 1>(3, 0) = rotation;
}
Twist::Twist(phri::Vector6d twist) : twist_(twist) {
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

/***			Acceleration			***/
Acceleration::Acceleration()
    : Acceleration(phri::Vector3d::Zero(), phri::Vector3d::Zero()) {
}
Acceleration::Acceleration(phri::Vector3d translation,
                           phri::Vector3d rotation) {
    acceleration_.block<3, 1>(0, 0) = translation;
    acceleration_.block<3, 1>(3, 0) = rotation;
}
Acceleration::Acceleration(phri::Vector6d acceleration)
    : acceleration_(acceleration) {
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

} // namespace phri
