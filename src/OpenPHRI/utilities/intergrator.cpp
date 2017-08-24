#include <OpenPHRI/utilities/integrator.hpp>

namespace phri {

template class Integrator<double>;
template class Integrator<Vector2d>;
template class Integrator<Vector3d>;
template class Integrator<Vector6d>;

}
