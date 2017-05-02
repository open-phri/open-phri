#include <RSCL/RSCL.h>

#include <boost/python.hpp>
#include <iostream>

namespace RSCL {

std::shared_ptr<SafetyController> NewSafetyController(Matrix6dPtr damping_matrix)
{
	return std::make_shared<SafetyController>(damping_matrix);
}

// Safety controller methods overloads
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SafetyController_addConstraint_overloads,        addConstraint,           2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SafetyController_addVelocityGenerator_overloads, addVelocityGenerator,    2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SafetyController_addForceGenerator_overloads,    addForceGenerator,       2, 3)

} // namespace RSCL

void wrapSafetyController() {
	using namespace RSCL;
	using namespace boost::python;

	/*********************************************************************************/
	/*                          SafetyController bindings                            */
	/*********************************************************************************/
	def("NewSafetyController",       &NewSafetyController,     "Create a new instance of a SafetyController shared_ptr");

	class_<SafetyController, boost::noncopyable>("SafetyController", "Generates a tool velocity based on force and velocity inputs and a set of constraints", no_init)
	.def("setVerbose",              &SafetyController::setVerbose)
	.def("addConstraint",           &SafetyController::addConstraint,           SafetyController_addConstraint_overloads(       args("name", "constraint", "force")))
	.def("addForceGenerator",       &SafetyController::addForceGenerator,       SafetyController_addForceGenerator_overloads(   args("name", "generator", "force")))
	.def("addVelocityGenerator",    &SafetyController::addVelocityGenerator,    SafetyController_addVelocityGenerator_overloads(args("name", "generator", "force")))
	.def("removeConstraint",        &SafetyController::removeConstraint)
	.def("removeForceGenerator",    &SafetyController::removeForceGenerator)
	.def("removeVelocityGenerator", &SafetyController::removeVelocityGenerator)
	.def("getConstraint",           &SafetyController::getConstraint)
	.def("getForceGenerator",       &SafetyController::getForceGenerator)
	.def("getVelocityGenerator",    &SafetyController::getVelocityGenerator)
	.def("updateTCPVelocity",       &SafetyController::updateTCPVelocity)
	.def("getTCPVelocity",          &SafetyController::getTCPVelocity)
	.def("getTotalVelocity",        &SafetyController::getTotalVelocity)
	.def("getTotalForce",           &SafetyController::getTotalForce);

	register_ptr_to_python<std::shared_ptr<SafetyController>>();
}
