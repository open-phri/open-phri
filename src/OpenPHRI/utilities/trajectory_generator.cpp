#include <OpenPHRI/utilities/trajectory_generator.h>

namespace phri {
template class TrajectoryGenerator<double>;
template class TrajectoryGenerator<VectorXd>;
template class TrajectoryGenerator<Vector6d>;
}
