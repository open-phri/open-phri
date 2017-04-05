#include <RSCL/RSCL.h>
#include "double_wrappers.h"

#include <boost/python.hpp>
#include <iostream>

namespace RSCL {

struct LinearInterpolatorWrap : public LinearInterpolator {
	using LinearInterpolator::LinearInterpolator;

	ConstDoubleWrap getOutputPy() {
		return ConstDoubleWrap(getOutput());
	}
};

struct PolynomialInterpolatorWrap : public PolynomialInterpolator {
	using PolynomialInterpolator::PolynomialInterpolator;

	ConstDoubleWrap getOutputPy() {
		return ConstDoubleWrap(getOutput());
	}
};


std::shared_ptr<LinearInterpolatorWrap> NewLinearInterpolator(Vector2dConstPtr from, Vector2dConstPtr to)
{
	return std::make_shared<LinearInterpolatorWrap>(from, to);
}

std::shared_ptr<LinearInterpolatorWrap> NewLinearInterpolator(Vector2dConstPtr from, Vector2dConstPtr to, doubleConstPtr input)
{
	return std::make_shared<LinearInterpolatorWrap>(from, to, input);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewLinearInterpolator_overloads, NewLinearInterpolator, 2, 3)

std::shared_ptr<PolynomialInterpolatorWrap> NewPolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to)
{
	return std::make_shared<PolynomialInterpolatorWrap>(from, to);
}

std::shared_ptr<PolynomialInterpolatorWrap> NewPolynomialInterpolator(PolynomialPointConstPtr from, PolynomialPointConstPtr to, doubleConstPtr input)
{
	return std::make_shared<PolynomialInterpolatorWrap>(from, to, input);
}
BOOST_PYTHON_FUNCTION_OVERLOADS(NewPolynomialInterpolator_overloads, NewPolynomialInterpolator, 2, 3)


} // namespace RSCL

void wrapInterpolators() {
	using namespace RSCL;
	using namespace boost::python;

	/*********************************************************************************/
	/*                            Interpolators bindings                             */
	/*********************************************************************************/
	def("NewLinearInterpolator",   (std::shared_ptr<LinearInterpolatorWrap>(*)(Vector2dConstPtr, Vector2dConstPtr, doubleConstPtr)) 0,
	    NewLinearInterpolator_overloads(
			args("from", "to", "rob_pos"),
			"Create a new instance of a LinearInterpolator shared_ptr"));

	def("NewPolynomialInterpolator",   (std::shared_ptr<PolynomialInterpolatorWrap>(*)(PolynomialPointConstPtr, PolynomialPointConstPtr, doubleConstPtr)) 0,
	    NewPolynomialInterpolator_overloads(
			args("from", "to", "rob_pos"),
			"Create a new instance of a PolynomialInterpolator shared_ptr"));

	struct InterpolatorWrap : Interpolator, boost::python::wrapper<Interpolator> {
		using Interpolator::Interpolator;

		void computeParameters()
		{
			this->get_override("computeParameters")();
		}

		double compute()
		{
			return this->get_override("compute")();
		}
	};


	class_<InterpolatorWrap, boost::noncopyable>("Interpolator", no_init)
	.def("setInput",            &InterpolatorWrap::setInput)
	.def("computeParameters",   pure_virtual(&InterpolatorWrap::computeParameters))
	.def("compute",             pure_virtual(&InterpolatorWrap::compute));

	class_<LinearInterpolatorWrap, boost::noncopyable, bases<InterpolatorWrap>>("LinearInterpolator", "Linear interpolator", no_init)
	.def("getOutput",           &LinearInterpolatorWrap::getOutputPy)
	.def("compute",             &LinearInterpolatorWrap::compute)
	.def("computeParameters",   &LinearInterpolatorWrap::computeParameters)
	.def("enableSaturation",    &LinearInterpolatorWrap::enableSaturation);

	class_<PolynomialPoint>("PolynomialPoint", "Interpolation point to use with PolynomialInterpolator", init<doubleConstPtr, doubleConstPtr, doubleConstPtr, doubleConstPtr>());
	class_<PolynomialInterpolator, boost::noncopyable, bases<Interpolator>>("PolynomialInterpolator", "5th order polynomial interpolator", no_init)
	.def("getOutput",           &PolynomialInterpolatorWrap::getOutputPy)
	.def("compute",             &PolynomialInterpolatorWrap::compute)
	.def("computeParameters",   &PolynomialInterpolatorWrap::computeParameters);

	register_ptr_to_python<std::shared_ptr<LinearInterpolatorWrap>>();
	register_ptr_to_python<std::shared_ptr<PolynomialPoint>>();
	register_ptr_to_python<std::shared_ptr<PolynomialInterpolator>>();

	implicitly_convertible<std::shared_ptr<LinearInterpolatorWrap>, std::shared_ptr<Interpolator>>();
	implicitly_convertible<std::shared_ptr<PolynomialInterpolator>, std::shared_ptr<Interpolator>>();

}
