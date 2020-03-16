#pragma once

#include <OpenPHRI/robot.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

class RobotModel {
public:
    RobotModel(Robot& robot, const std::string& model_path,
               const std::string& control_point);
    RobotModel(Robot& robot, const YAML::Node& configuration);
    RobotModel(const RobotModel& other);
    RobotModel(RobotModel&& other) noexcept;
    ~RobotModel();

    RobotModel& operator=(const RobotModel& other);
    RobotModel& operator=(RobotModel&& other) noexcept;

    void forwardKinematics() const;

    std::shared_ptr<const Eigen::VectorXd> getLowerLimits() const;
    std::shared_ptr<const Eigen::VectorXd> getUpperLimits() const;
    std::shared_ptr<const Eigen::VectorXd> getVelocityLimits() const;
    std::shared_ptr<const Eigen::VectorXd> getForceLimits() const;

    size_t jointCount() const;
    const std::string& name() const;

private:
    struct pImpl;
    std::unique_ptr<pImpl> impl_;
};

} // namespace phri
