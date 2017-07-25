#include <RSCL/utilities/demangle.h>

#include <typeinfo>
#include <memory>
#include <cxxabi.h>

std::string RSCL::demangle(const char* name) {
	int status = -4; // some arbitrary value to eliminate the compiler warning

	std::unique_ptr<char, void (*)(void*)> res {
		abi::__cxa_demangle(name, NULL, NULL, &status),
		std::free
	};

	return (status==0) ? res.get() : name;
}
