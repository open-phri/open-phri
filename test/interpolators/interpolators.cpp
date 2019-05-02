#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

using namespace phri;
using namespace std;
using namespace Eigen;

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

int main(int argc, char const* argv[]) {

    auto lin_from = make_shared<LinearPoint>(0., 1.);
    auto lin_to = make_shared<LinearPoint>(1., 2.);
    auto lin_x = make_shared<double>(0.);

    auto lin_interp = LinearInterpolator(lin_from, lin_to, lin_x);

    auto lin_y = lin_interp.getOutput();

    *lin_x = *lin_from->x;
    lin_interp.compute();
    assert_msg("Step #1-1", isClose(*lin_y, *lin_from->y));

    *lin_x = *lin_to->x;
    lin_interp.compute();
    assert_msg("Step #1-2", isClose(*lin_y, *lin_to->y));

    *lin_x = 2.;
    lin_interp.compute();
    assert_msg("Step #1-3", isClose(*lin_y, 3.));

    *lin_x = -2.;
    lin_interp.compute();
    assert_msg("Step #1-4", isClose(*lin_y, -1.));

    lin_interp.enableSaturation(true);
    lin_interp.compute();
    assert_msg("Step #1-5", isClose(*lin_y, *lin_from->y));

    *lin_x = 2.;
    lin_interp.compute();
    assert_msg("Step #1-6", isClose(*lin_y, *lin_to->y));

    *lin_x = 0.5;
    lin_interp.compute();
    assert_msg("Step #1-4", isClose(*lin_y, 1.5));

    auto poly_from = make_shared<PolynomialPoint>(0., // x
                                                  1., // y
                                                  0., // dy
                                                  0.  // d2y
    );
    auto poly_to = make_shared<PolynomialPoint>(1., // x
                                                2., // y
                                                0., // dy
                                                0.  // d2y
    );
    auto poly_x = make_shared<double>(0.);

    auto poly_interp = PolynomialInterpolator(poly_from, poly_to, poly_x);

    auto poly_y = poly_interp.getOutput();

    *poly_x = *poly_from->x;
    poly_interp.compute();
    assert_msg("Step #2-1", isClose(*poly_y, *poly_from->y));

    *poly_x = *poly_to->x;
    poly_interp.compute();
    assert_msg("Step #2-2", isClose(*poly_y, *poly_to->y));

    *poly_x = 2.;
    poly_interp.compute();
    assert_msg("Step #2-3", isClose(*poly_y, *poly_to->y));

    *poly_x = -2.;
    poly_interp.compute();
    assert_msg("Step #2-4", isClose(*poly_y, *poly_from->y));

    *poly_x = 0.5;
    poly_interp.compute();
    assert_msg("Step #2-4", isClose(*poly_y, 1.5));

    return 0;
}
