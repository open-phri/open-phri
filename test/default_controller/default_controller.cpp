#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>
#include <pid/rpath.h>

using namespace phri;
using namespace std;

int main(int argc, char const* argv[]) {
    using namespace spatial::literals;

    auto robot = phri::Robot{"tcp"_frame, "base"_frame, "rob", 7};

    auto model = phri::RobotModel(
        robot, PID_PATH("robot_models/kuka_lwr4.yaml"), "end-effector");

    robot.joints().state.position.setOnes();
    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    safety_controller.compute();

    assert(robot.task().command.twist.vector().isZero());

    return 0;
}
