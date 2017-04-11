#pragma once

#include <Eigen/Dense>
#include <memory>

namespace RSCL {

#define assert_msg(msg,cond) assert(((void) msg, cond ))


/***		Useful typedefs		***/
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Matrix6dPtr = std::shared_ptr<Matrix6d>;
using Vector2dPtr = std::shared_ptr<Vector2d>;
using Vector3dPtr = std::shared_ptr<Vector3d>;
using Vector6dPtr = std::shared_ptr<Vector6d>;
using doublePtr = std::shared_ptr<double>;

using Matrix6dConstPtr = std::shared_ptr<const Matrix6d>;
using Vector2dConstPtr = std::shared_ptr<const Vector2d>;
using Vector3dConstPtr = std::shared_ptr<const Vector3d>;
using Vector6dConstPtr = std::shared_ptr<const Vector6d>;
using doubleConstPtr = std::shared_ptr<const double>;

}
