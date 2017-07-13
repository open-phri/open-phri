/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of RSCL <https://gite.lirmm.fr/navarro/RSCL>.
 *
 *  RSCL is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  RSCL is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with RSCL.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file definitions.h
 * @author Benjamin Navarro
 * @brief Usefull definitions being used in RSCL
 * @date April 2017
 * @ingroup RSCL
 */

#pragma once

#include <Eigen/Dense>
#include <memory>

namespace RSCL {

#define assert_msg(msg,cond) assert(((void) msg, cond ))

/** @enum RSCL::ReferenceFrame
 *  @brief Specify an object's reference frame.
 */
enum class ReferenceFrame {
	TCP,    /**< Controlled frame. Tool control frame */
	Base,   /**< Frame fixed relative to the robot's base */
	World   /**< Frame fixed relative to the environment */
};

/***		Useful typedefs		***/
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Eigen::VectorXd;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector4d = Eigen::Matrix<double, 4, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

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

}
