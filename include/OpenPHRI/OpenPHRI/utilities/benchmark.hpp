/*
 *  Copyright (C) 2017 Benjamin Navarro <contact@bnavarro.info>
 *
 *  This file is part of OpenPHRI <https://gite.lirmm.fr/navarro/OpenPHRI>.
 *
 *  OpenPHRI is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU Lesser General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  OpenPHRI is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU Lesser General Public License
 *  along with OpenPHRI.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
 * @file benchmark.hpp
 * @author Benjamin Navarro
 * @brief Some helper functions to benchmark functions execution time
 * @date June 2017
 * @ingroup OpenPHRI
 */

#pragma once

#include <functional>
#include <type_traits>

namespace OpenPHRI {

template<int N>
inline void run_function_once(std::function<void (void)> todo) {
	todo();
	run_function_once<N-1>(todo);
}

template<>
inline void run_function_once<0>(std::function<void(void)> todo) {
	return;
}

constexpr size_t __MAX_TEMPLATE_INSTANCIATION_DEPTH = 256;

template<int N, typename std::enable_if<(N <= __MAX_TEMPLATE_INSTANCIATION_DEPTH), bool>::type = 0>
inline void run_function(std::function<void(void)> todo) {
	run_function_once<N>(todo);
}

// Use a loop if N>__MAX_TEMPLATE_INSTANCIATION_DEPTH to avoid reaching the template maximum instantiation depth. (should be 1024 for C++11 but gcc set it to 900 and clang 3.9 to 256 (v4.0 is ok))
template<int N, typename std::enable_if<not (N <= __MAX_TEMPLATE_INSTANCIATION_DEPTH), bool>::type = 0> // use <= to remove parsing error in atom when using >
inline void run_function(std::function<void(void)> todo) {
	size_t iter = N;
	while(iter >= __MAX_TEMPLATE_INSTANCIATION_DEPTH) {
		run_function_once<__MAX_TEMPLATE_INSTANCIATION_DEPTH>(todo);
		iter -= __MAX_TEMPLATE_INSTANCIATION_DEPTH;
	}
	run_function_once<N%__MAX_TEMPLATE_INSTANCIATION_DEPTH>(todo);
}


} // namespace OpenPHRI
