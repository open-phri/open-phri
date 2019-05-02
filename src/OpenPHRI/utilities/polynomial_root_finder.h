/*      File: polynomial_root_finder.h
 *       This file is part of the program open-phri
 *       Program description : OpenPHRI: a generic framework to easily and
 * safely control robots in interactions with humans Copyright (C) 2017 -
 * Benjamin Navarro (LIRMM). All Right reserved.
 *
 *       This software is free software: you can redistribute it and/or modify
 *       it under the terms of the LGPL license as published by
 *       the Free Software Foundation, either version 3
 *       of the License, or (at your option) any later version.
 *       This software is distributed in the hope that it will be useful,
 *       but WITHOUT ANY WARRANTY without even the implied warranty of
 *       MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 *       LGPL License for more details.
 *
 *       You should have received a copy of the GNU Lesser General Public
 * License version 3 and the General Public License version 3 along with this
 * program. If not, see <http://www.gnu.org/licenses/>.
 */

// rpoly_ak1.cpp - Program for calculating the roots of a polynomial of real
// coefficients. Written in Visual C++ 2005 Express Edition 14 July 2007
//
// The sub-routines listed below are translations of the FORTRAN routines
// included in RPOLY.FOR, posted off the NETLIB site as TOMS/493:
//
// http://www.netlib.org/toms/493
//
// TOMS/493 is based on the Jenkins-Traub algorithm.
//
// To distinguish the routines posted below from others, an _ak1 suffix has been
// appended to them.
//
// Following is a list of the major changes made in the course of translating
// the TOMS/493 routines to the C++ versions posted below: 1) All global
// variables have been eliminated. 2) The "FAIL" parameter passed into RPOLY.FOR
// has been eliminated. 3) RPOLY.FOR solves polynomials of degree up to 100, but
// does not explicitly state this limit.
//     rpoly_ak1 explicitly states this limit; uses the macro name MAXDEGREE to
//     specify this limit; and does a check to ensure that the user input
//     variable Degree is not greater than MAXDEGREE (if it is, an error message
//     is output and rpoly_ak1 terminates). If a user wishes to compute roots of
//     polynomials of degree greater than MAXDEGREE, using a macro name like
//     MAXDEGREE provides the simplest way of offering this capability.
// 4) All "GO TO" statements have been eliminated.
//
// A small main program is included also, to provide an example of how to use
// rpoly_ak1. In this example, data is input from a file to eliminate the need
// for a user to type data in via the console.

constexpr int MAXDEGREE = 3;
constexpr int MDP1 = MAXDEGREE + 1;

void rpoly_ak1(double op[MDP1], int* Degree, double zeror[MAXDEGREE],
               double zeroi[MAXDEGREE]);
