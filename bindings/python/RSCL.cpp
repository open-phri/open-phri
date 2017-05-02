#include <RSCL/RSCL.h>
#include <boost/python.hpp>

extern void wrapMisc();
extern void wrapConstraints();
extern void wrapGenerators();
extern void wrapInterpolators();
extern void wrapUtilities();
extern void wrapSafetyController();
extern void wrapVREP();

BOOST_PYTHON_MODULE(PyRSCL) {
	using namespace RSCL;

	using namespace boost::python;

	wrapMisc();
	wrapUtilities();
	wrapConstraints();
	wrapGenerators();
	wrapInterpolators();
	wrapSafetyController();
	wrapVREP();
}
