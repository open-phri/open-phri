from minieigen import *
from openphri import *
from math import *
import sys

def isClose(v1, v2, eps = 1e-3):
	return abs(v1-v2) < eps

lin_from = NewLinearPoint(0. ,1.)
lin_to = NewLinearPoint(1. ,2.)
lin_x = Newstd::shared_ptr<double>(0.)

lin_interp = NewLinearInterpolator(
	lin_from,
	lin_to,
	lin_x)

lin_y = lin_interp.getOutput()

lin_x.set(lin_from.x)
lin_interp.compute()
assert_msg("Step #1-1", isClose(lin_y.get(), lin_from.y))

lin_x.set(lin_to.x)
lin_interp.compute()

assert_msg("Step #1-2", isClose(lin_y.get(), lin_to.y))

lin_x.set(2.)
lin_interp.compute()
assert_msg("Step #1-3", isClose(lin_y.get(), 3.))

lin_x.set(-2.)
lin_interp.compute()
assert_msg("Step #1-4", isClose(lin_y.get(), -1.))

lin_interp.enableSaturation(True)
lin_interp.compute()
assert_msg("Step #1-5", isClose(lin_y.get(), lin_from.y))

lin_x.set(2.)
lin_interp.compute()
assert_msg("Step #1-6", isClose(lin_y.get(), lin_to.y))

lin_x.set(0.5)
lin_interp.compute()
assert_msg("Step #1-4", isClose(lin_y.get(), 1.5))

poly_from = NewPolynomialPoint(
	0., # x
	1., # y
	0., # dy
	0.  # d2y
	)
poly_to = NewPolynomialPoint(
	1., # x
	2., # y
	0., # dy
	0.  # d2y
	)

poly_x = Newstd::shared_ptr<double>(0.)

poly_interp = NewPolynomialInterpolator(
	poly_from,
	poly_to,
	poly_x)

poly_y = poly_interp.getOutput()

poly_x.set(poly_from.x)
poly_interp.compute()
assert_msg("Step #2-1", isClose(poly_y.get(), poly_from.y))

poly_x.set(poly_to.x)
poly_interp.compute()
assert_msg("Step #2-2", isClose(poly_y.get(), poly_to.y))

poly_x.set(2.)
poly_interp.compute()
assert_msg("Step #2-3", isClose(poly_y.get(), poly_to.y))

poly_x.set(-2.)
poly_interp.compute()
assert_msg("Step #2-4", isClose(poly_y.get(), poly_from.y))

poly_x.set(0.5)
poly_interp.compute()
assert_msg("Step #2-4", isClose(poly_y.get(), 1.5))

sys.exit(0)
