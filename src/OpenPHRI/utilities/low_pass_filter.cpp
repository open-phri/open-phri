#include <OpenPHRI/utilities/low_pass_filter.hpp>

namespace phri {

template class LowPassFilter<double>;
template class LowPassFilter<Eigen::Vector2d>;
template class LowPassFilter<Eigen::Vector3d>;
template class LowPassFilter<Eigen::Vector6d>;
template class LowPassFilter<Eigen::VectorXd>;

} // namespace phri
