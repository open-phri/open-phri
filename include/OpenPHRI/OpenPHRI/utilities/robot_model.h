#pragma once

#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

class RobotModel {
public:
    RobotModel(RobotPtr robot, const std::string& model_path,
               const std::string& control_point);
    RobotModel(RobotPtr robot, const YAML::Node& configuration);
    ~RobotModel();

    void forwardKinematics() const;

    VectorXdConstPtr getLowerLimits() const;
    VectorXdConstPtr getUpperLimits() const;
    VectorXdConstPtr getVelocityLimits() const;
    VectorXdConstPtr getForceLimits() const;

    size_t jointCount() const;
    const std::string& name() const;

private:
    struct pImpl;
    std::unique_ptr<pImpl> impl_;
};

using RobotModelPtr = std::shared_ptr<RobotModel>;
using RobotModelConstPtr = std::shared_ptr<const RobotModel>;

} // namespace phri
