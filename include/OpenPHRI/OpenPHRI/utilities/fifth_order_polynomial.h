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
 * @file fifth_order_polynomial.h
 * @author Benjamin Navarro
 * @brief Definition of FifthOrderPolynomial class
 * @date April 2017
 * @ingroup OpenPHRI
 */

#pragma once

namespace phri {

class FifthOrderPolynomial {
public:
	struct Parameters {
		// User defined constraints
		double xi;
		double xf;
		double yi;
		double yf;
		double dyi;
		double dyf;
		double d2yi;
		double d2yf;

		// Polynomial coefficients
		double a;
		double b;
		double c;
		double d;
		double e;
		double f;
	};

	static void computeParameters(FifthOrderPolynomial::Parameters& params);
	static double compute(double x, const Parameters& params);
	static double computeFirstDerivative(double x, const Parameters& params);
	static double computeSecondDerivative(double x, const Parameters& params);
	static double getFirstDerivativeMaximum(const Parameters& params);
	static double getSecondDerivativeMaximum(const Parameters& params);

private:
	FifthOrderPolynomial() = default;
	~FifthOrderPolynomial() = default;
};

} // namespace OpenPHRI;
