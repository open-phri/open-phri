#include <RSCL/RSCL.h>

#include <boost/python.hpp>
#include <iostream>

namespace RSCL {

std::shared_ptr<VelocityProxy> NewVelocityProxy(Vector6dPtr velocity)
{
	return std::make_shared<VelocityProxy>(static_cast<Vector6dConstPtr>(velocity));
}

std::shared_ptr<ForceProxy> NewForceProxy(Vector6dPtr force)
{
	return std::make_shared<ForceProxy>(static_cast<Vector6dConstPtr>(force));
}

std::shared_ptr<PotentialFieldObject> NewPotentialFieldObject(PotentialFieldType type, doublePtr gain, doublePtr threshold_distance, Vector6dPtr object_position)
{
	return std::make_shared<PotentialFieldObject>(type, static_cast<doubleConstPtr>(gain), static_cast<doubleConstPtr>(threshold_distance), static_cast<Vector6dConstPtr>(object_position));
}

std::shared_ptr<PotentialFieldGenerator> NewPotentialFieldGenerator(Vector6dPtr robot_position)
{
	return std::make_shared<PotentialFieldGenerator>(static_cast<Vector6dConstPtr>(robot_position));
}

std::shared_ptr<PotentialFieldGenerator> NewPotentialFieldGenerator()
{
	return std::make_shared<PotentialFieldGenerator>();
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewPotentialFieldGenerator_overloads, NewPotentialFieldGenerator, 0, 1)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(PotentialFieldGenerator_addObject_overloads, addObject, 2, 3)

} // namespace RSCL

void wrapGenerators() {
	using namespace RSCL;
	using namespace boost::python;

	/**********************************************************************************/
	/*                                 Generators bindings                            */
	/**********************************************************************************/
	def("NewVelocityProxy",             NewVelocityProxy,           "Create a new instance of a VelocityProxy shared_ptr");
	def("NewForceProxy",                NewForceProxy,              "Create a new instance of a ForceProxy shared_ptr");
	def("NewPotentialFieldObject",      NewPotentialFieldObject,    "Create a new instance of a PotentialFieldObject shared_ptr");
	def("NewPotentialFieldGenerator",   (std::shared_ptr<PotentialFieldGenerator>(*)(Vector6dPtr)) 0,
	    NewPotentialFieldGenerator_overloads(args("rob_pos"),       "Create a new instance of a PotentialFieldGenerator shared_ptr"));

	struct VelocityGeneratorWrap : VelocityGenerator, wrapper<VelocityGenerator> {
		using VelocityGenerator::VelocityGenerator;

		Vector6d compute()
		{
			return this->get_override("compute")();
		}
	};

	class_<VelocityGeneratorWrap, boost::noncopyable>("VelocityGenerator", no_init)
	.def("compute", pure_virtual(&VelocityGenerator::compute));

	class_<VelocityProxy, boost::noncopyable>("VelocityProxy", no_init)
	.def("compute", &VelocityProxy::compute);


	struct ForceGeneratorWrap : ForceGenerator, wrapper<ForceGenerator> {
		using ForceGenerator::ForceGenerator;

		Vector6d compute()
		{
			return this->get_override("compute")();
		}
	};

	class_<ForceGeneratorWrap, boost::noncopyable>("ForceGenerator", no_init)
	.def("compute", pure_virtual(&ForceGenerator::compute));

	class_<ForceProxy, boost::noncopyable>("ForceProxy", no_init)
	.def("compute", &ForceProxy::compute);

	register_ptr_to_python<std::shared_ptr<VelocityProxy>>();
	register_ptr_to_python<std::shared_ptr<ForceProxy>>();

	implicitly_convertible<std::shared_ptr<VelocityProxy>, std::shared_ptr<VelocityGenerator>>();
	implicitly_convertible<std::shared_ptr<ForceProxy>,    std::shared_ptr<ForceGenerator>>();

	enum_<PotentialFieldType>("PotentialFieldType", "Type of potential field")
	.value("Attractive", PotentialFieldType::Attractive)
	.value("Repulsive", PotentialFieldType::Repulsive);

	class_<PotentialFieldObject>("PotentialFieldObject", init<PotentialFieldType, doubleConstPtr, doubleConstPtr, Vector6dConstPtr>());

	class_<PotentialFieldGenerator, boost::noncopyable, bases<ForceGenerator>>("PotentialFieldGenerator", no_init)
	.def("setVerbose",      &PotentialFieldGenerator::setVerbose)
	.def("compute",         &PotentialFieldGenerator::compute)
	.def("addObject",       &PotentialFieldGenerator::addObject,           PotentialFieldGenerator_addObject_overloads(args("name", "object", "force")))
	.def("removeObject",    &PotentialFieldGenerator::removeObject)
	.def("getObject",       &PotentialFieldGenerator::getObject);

	register_ptr_to_python<std::shared_ptr<PotentialFieldObject>>();
	register_ptr_to_python<std::shared_ptr<PotentialFieldGenerator>>();
	implicitly_convertible<std::shared_ptr<PotentialFieldGenerator>,    std::shared_ptr<ForceGenerator>>();

}
