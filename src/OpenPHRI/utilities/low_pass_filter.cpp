#include <OpenPHRI/utilities/low_pass_filter.hpp>

namespace phri {

template class LowPassFilter<double>;
template class LowPassFilter<Vector2d>;
template class LowPassFilter<Vector3d>;
template class LowPassFilter<Vector6d>;
template class LowPassFilter<VectorXd>;

} // namespace phri
