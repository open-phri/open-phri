#include <OpenPHRI/OpenPHRI.h>
#include <vrep_driver/vrep_driver.h>

#include <boost/python.hpp>
#include <iostream>

namespace phri {

using namespace vrep;

std::shared_ptr<VREPDriver> NewVREPDriver(double sample_time, const std::string& prefix = "", const std::string& suffix = "", const std::string& ip = "127.0.0.1", int port = 19997)
{
	return std::make_shared<VREPDriver>(sample_time, prefix, suffix, ip, port);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewVREPDriver_overloads, NewVREPDriver, 1, 5)

} // namespace phri

void wrapVREP() {
	using namespace phri;
	using namespace vrep;
	using namespace boost::python;

	/*********************************************************************************/
	/*                               VREPDriver bindings                             */
	/*********************************************************************************/
	enum_<ReferenceFrame>("ReferenceFrame", "Frame of reference")
	.value("TCP", ReferenceFrame::TCP)
	.value("Base", ReferenceFrame::Base)
	.value("World", ReferenceFrame::World);

	def("NewVREPDriver",
	    NewVREPDriver,
	    NewVREPDriver_overloads(
			args("sample_time", "prefix", "suffix", "ip", "port"), "Create a new instance of a VREPDriver shared_ptr"));

	class_<VREPDriver, boost::noncopyable>("VREPDriver", "Interface to the V-REP simulator", init<double, const std::string&, const std::string&, const std::string&, int>())
	.def(init<double, int, const std::string&, const std::string&>())
	.def("getClientID",                     &VREPDriver::getClientID)
	.def("checkConnection",                 &VREPDriver::checkConnection)
	.def("enableSynchonous",                &VREPDriver::enableSynchonous)
	.def("nextStep",                        &VREPDriver::nextStep)
	.def("startSimulation",                 &VREPDriver::startSimulation)
	.def("stopSimulation",                  &VREPDriver::stopSimulation)
	.def("pauseSimulation",                 &VREPDriver::pauseSimulation)
	.def("readTCPPose",                     &VREPDriver::readTCPPose)
	.def("readTCPVelocity",                 &VREPDriver::readTCPVelocity)
	.def("readTCPTargetPose",               &VREPDriver::readTCPTargetPose)
	.def("sendTCPtargetVelocity",           &VREPDriver::sendTCPtargetVelocity)
	.def("readTCPWrench",                   &VREPDriver::readTCPWrench)
	.def("trackObjectPosition",             &VREPDriver::trackObjectPosition)
	.def("updateTrackedObjectsPosition",    &VREPDriver::updateTrackedObjectsPosition)
	;

	register_ptr_to_python<std::shared_ptr<VREPDriver>>();
}
