/*      File: type_aliases.h
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
 * @file type_aliases.h
 * @author Benjamin Navarro
 * @brief Usefull type aliases being used in OpenPHRI
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <Eigen/Dense>
#include <memory>

namespace phri {

using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::MatrixXd;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;
using Eigen::VectorXd;
using Vector6d = Eigen::Matrix<double, 6, 1>;
using AffineTransform = Eigen::Transform<double, 3, Eigen::Affine>;

using Matrix3dPtr = std::shared_ptr<Eigen::Matrix3d>;
using Matrix4dPtr = std::shared_ptr<Eigen::Matrix4d>;
using Matrix6dPtr = std::shared_ptr<Matrix6d>;
using MatrixXdPtr = std::shared_ptr<Eigen::MatrixXd>;
using Vector2dPtr = std::shared_ptr<Vector2d>;
using Vector3dPtr = std::shared_ptr<Vector3d>;
using Vector4dPtr = std::shared_ptr<Vector4d>;
using Vector6dPtr = std::shared_ptr<Vector6d>;
using VectorXdPtr = std::shared_ptr<Eigen::VectorXd>;
using doublePtr = std::shared_ptr<double>;
using AffineTransformPtr = std::shared_ptr<AffineTransform>;

using Matrix3dConstPtr = std::shared_ptr<const Eigen::Matrix3d>;
using Matrix4dConstPtr = std::shared_ptr<const Eigen::Matrix4d>;
using Matrix6dConstPtr = std::shared_ptr<const Matrix6d>;
using MatrixXdConstPtr = std::shared_ptr<const Eigen::MatrixXd>;
using Vector2dConstPtr = std::shared_ptr<const Vector2d>;
using Vector3dConstPtr = std::shared_ptr<const Vector3d>;
using Vector4dConstPtr = std::shared_ptr<const Vector4d>;
using Vector6dConstPtr = std::shared_ptr<const Vector6d>;
using VectorXdConstPtr = std::shared_ptr<const Eigen::VectorXd>;
using doubleConstPtr = std::shared_ptr<const double>;
using AffineTransformConstPtr = std::shared_ptr<const AffineTransform>;

} // namespace phri
