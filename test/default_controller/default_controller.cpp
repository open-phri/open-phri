#include <OpenPHRI/OpenPHRI.h>
#include "utils.h"

int main(int argc, char const* argv[]) {
    using namespace spatial::literals;

    auto [robot, model, driver] = TestData{};

    driver.jointState().position().setOnes();

    model.forwardKinematics();

    auto safety_controller = phri::SafetyController(robot);
    safety_controller.setVerbose(true);

    safety_controller.compute();

    assert(robot.task().command().velocity().isZero());

    return 0;
}
