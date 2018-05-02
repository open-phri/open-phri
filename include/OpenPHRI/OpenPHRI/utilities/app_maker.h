#pragma once

#include <OpenPHRI/OpenPHRI.h>
#include <OpenPHRI/drivers/driver.h>
#include <OpenPHRI/fwd_decl.h>

namespace phri {

class AppMaker {
public:
	AppMaker(const std::string& configuration_file);
	~AppMaker();

	bool init(std::function<bool(void)> init_code = std::function<bool(void)>());

	bool run(
		std::function<bool(void)> pre_controller_code = std::function<bool(void)>(),
		std::function<bool(void)> post_controller_code = std::function<bool(void)>());

	bool stop();

	RobotPtr getRobot() const;
	SafetyControllerPtr getController() const;
	RobotModelPtr getModel() const;
	DriverPtr getDriver() const;
	DataLoggerPtr getDataLogger() const;

private:
	RobotPtr robot_;
	SafetyControllerPtr controller_;
	RobotModelPtr model_;
	DriverPtr driver_;
	DataLoggerPtr data_logger_;
	ClockPtr clock_;
};

using AppMakerPtr = std::shared_ptr<AppMaker>;
using AppMakerConstPtr = std::shared_ptr<const AppMaker>;

}
