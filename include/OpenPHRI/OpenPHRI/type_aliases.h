
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

using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector4d = Eigen::Matrix<double, 4, 1>;
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
