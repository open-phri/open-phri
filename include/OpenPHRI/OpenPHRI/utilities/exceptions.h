#pragma once

#include <exception>
#include <string>

namespace phri {

// Original solution found at http://stackoverflow.com/questions/37181621/easy-way-of-constructing-information-message-for-throwing-stdexception-using-p
inline std::string make_open_phri_error(const std::string& msg,
                                        char const* file,
                                        char const* function,
                                        std::size_t line)
{
	return std::string{"OpenPHRI error in "} +file + ":" + std::to_string(line) + " [" +
	       function + "]: " + msg;
}

#ifdef _MSC_VER
    #define OPEN_PHRI_PRETTY_FUNCTION __FUNCTION__ 
#else
    #define OPEN_PHRI_PRETTY_FUNCTION __PRETTY_FUNCTION__
#endif

#define OPEN_PHRI_ERROR(...) phri::make_open_phri_error(__VA_ARGS__, __FILE__, OPEN_PHRI_PRETTY_FUNCTION, __LINE__ )

} // namespace phri
