#ifndef DEFINITIONS_H
#define DEFINITIONS_H

#include <Eigen/Dense>
#include <memory>

namespace RSCL {

/***		Useful typedefs		***/
using Matrix6d = Eigen::Matrix<double, 6, 6>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

using Matrix6dPtr = std::shared_ptr<Matrix6d>;
using Vector6dPtr = std::shared_ptr<Vector6d>;
using doublePtr = std::shared_ptr<double>;

using Matrix6dConstPtr = std::shared_ptr<const Matrix6d>;
using Vector6dConstPtr = std::shared_ptr<const Vector6d>;
using doubleConstPtr = std::shared_ptr<const double>;

}

#endif /* DEFINITIONS_H */
