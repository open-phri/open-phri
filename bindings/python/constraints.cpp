#include <RSCL/RSCL.h>
#include "double_wrappers.h"

#include <boost/python.hpp>
#include <iostream>

namespace RSCL {

// Wrappers for classes returning pointers to double
struct PowerConstraintWrap : public PowerConstraint {
	using PowerConstraint::PowerConstraint;

	ConstDoubleWrap getPowerPy() {
		return ConstDoubleWrap(getPower());
	}
};


// Functions to create shared pointers to constraints
std::shared_ptr<DefaultConstraint> NewDefaultConstraint(ConstraintType type)
{
	return std::make_shared<DefaultConstraint>(type);
}

std::shared_ptr<VelocityConstraint> NewVelocityConstraint(Vector6dPtr total_velocity, doubleConstPtr maximum_velocity)
{
	return std::make_shared<VelocityConstraint>(static_cast<Vector6dConstPtr>(total_velocity), static_cast<doubleConstPtr>(maximum_velocity));
}

std::shared_ptr<PowerConstraintWrap> NewPowerConstraint(Vector6dPtr total_velocity, Vector6dPtr external_force, doublePtr maximum_power)
{
	return std::make_shared<PowerConstraintWrap>(static_cast<Vector6dConstPtr>(total_velocity), static_cast<Vector6dConstPtr>(external_force), static_cast<doubleConstPtr>(maximum_power));
}

std::shared_ptr<StopConstraint> NewStopConstraint(Vector6dPtr external_force, doublePtr activation_force_threshold, doublePtr deactivation_force_threshold)
{
	return std::make_shared<StopConstraint>(static_cast<Vector6dConstPtr>(external_force), static_cast<doubleConstPtr>(activation_force_threshold), static_cast<doubleConstPtr>(deactivation_force_threshold));
}

std::shared_ptr<SeparationDistanceConstraint> NewSeparationDistanceConstraint(ConstraintPtr constraint, InterpolatorPtr interpolator)
{
	return std::make_shared<SeparationDistanceConstraint>(constraint, interpolator);
}

std::shared_ptr<SeparationDistanceConstraint> NewSeparationDistanceConstraint(ConstraintPtr constraint, InterpolatorPtr interpolator, Vector6dConstPtr robot_position)
{
	return std::make_shared<SeparationDistanceConstraint>(constraint, interpolator, robot_position);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewSeparationDistanceConstraint_overloads, NewSeparationDistanceConstraint, 2, 3)
BOOST_PYTHON_MEMBER_FUNCTION_OVERLOADS(SeparationDistanceConstraint_addObject_overloads, addObject, 2, 3)


} // namespace RSCL

void wrapConstraints() {
	using namespace RSCL;
	using namespace boost::python;


	/**********************************************************************************/
	/*                                 Constraint bindings                            */
	/**********************************************************************************/
	def("NewDefaultConstraint",            NewDefaultConstraint,          "Create a new instance of a DefaultConstraint shared_ptr");
	def("NewVelocityConstraint",           NewVelocityConstraint,         "Create a new instance of a VelocityConstraint shared_ptr");
	def("NewPowerConstraint",              NewPowerConstraint,            "Create a new instance of a PowerConstraint shared_ptr");
	def("NewStopConstraint",               NewStopConstraint,             "Create a new instance of a StopConstraint shared_ptr");
	def("NewSeparationDistanceConstraint", (std::shared_ptr<SeparationDistanceConstraint>(*)(ConstraintPtr, InterpolatorPtr, Vector6dConstPtr)) 0,
	    NewSeparationDistanceConstraint_overloads(
			args("constraint", "interpolator", "robot_position"),
			"Create a new instance of a SeparationDistanceConstraint shared_ptr"));

	enum_<ConstraintType>("ConstraintsType", "Type of constraint")
	.value("Multiplicative", ConstraintType::Multiplicative)
	.value("Minimum", ConstraintType::Minimum);

	struct ConstraintWrap : Constraint, wrapper<Constraint> {
		using Constraint::Constraint;

		double compute()
		{
			return this->get_override("compute")();
		}
	};

	class_<ConstraintWrap, boost::noncopyable>("Constraint", no_init)
	.def("getType", &Constraint::getType)
	.def("compute", pure_virtual(&Constraint::compute));

	class_<DefaultConstraint, boost::noncopyable, bases<Constraint>>("DefaultConstraint", "Default constraint, always evaluates to 1", no_init)
	.def("compute", &DefaultConstraint::compute);

	class_<VelocityConstraint, boost::noncopyable, bases<Constraint>>("VelocityConstraint", "Velocity constraint, limits the tool's maximum velocity", no_init)
	.def("compute", &VelocityConstraint::compute);

	class_<PowerConstraintWrap, boost::noncopyable, bases<Constraint>>("PowerConstraint", "Power constraint, limits the exchanged power between the tool and the environment", no_init)
	.def("compute", &PowerConstraintWrap::compute)
	.def("getPower", &PowerConstraintWrap::getPowerPy);

	class_<StopConstraint, boost::noncopyable, bases<Constraint>>("StopConstraint", "Stop constraint, stops the tool when the external force is above a given limit", no_init)
	.def("compute", &StopConstraint::compute);

	class_<SeparationDistanceConstraint, boost::noncopyable, bases<Constraint>>("SeparationDistanceConstraint", "Separation distance constraint, adapts the constraint depending on the distance to the closest object", no_init)
	.def("compute",         &SeparationDistanceConstraint::compute)
	.def("setVerbose",      &SeparationDistanceConstraint::setVerbose)
	.def("addObject",       &SeparationDistanceConstraint::addObject, SeparationDistanceConstraint_addObject_overloads(args("name", "object", "force")))
	.def("removeObject",    &SeparationDistanceConstraint::removeObject)
	.def("getObject",       &SeparationDistanceConstraint::getObject);

	register_ptr_to_python<std::shared_ptr<DefaultConstraint>>();
	register_ptr_to_python<std::shared_ptr<VelocityConstraint>>();
	register_ptr_to_python<std::shared_ptr<PowerConstraintWrap>>();
	register_ptr_to_python<std::shared_ptr<StopConstraint>>();
	register_ptr_to_python<std::shared_ptr<SeparationDistanceConstraint>>();

	implicitly_convertible<std::shared_ptr<DefaultConstraint>,              std::shared_ptr<Constraint>>();
	implicitly_convertible<std::shared_ptr<VelocityConstraint>,             std::shared_ptr<Constraint>>();
	implicitly_convertible<std::shared_ptr<PowerConstraintWrap>,            std::shared_ptr<Constraint>>();
	implicitly_convertible<std::shared_ptr<StopConstraint>,                 std::shared_ptr<Constraint>>();
	implicitly_convertible<std::shared_ptr<SeparationDistanceConstraint>,   std::shared_ptr<Constraint>>();

}
