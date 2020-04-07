#undef NDEBUG

#include <OpenPHRI/OpenPHRI.h>

bool isClose(double v1, double v2, double eps = 1e-3) {
    return std::abs(v1 - v2) < eps;
}

int main(int argc, char const* argv[]) {
    {
        using Interpolator = phri::LinearInterpolator<double, double>;

        auto lin_from = Interpolator::Point{0., 1.};
        auto lin_to = Interpolator::Point{1., 2.};
        auto lin_x = std::make_shared<double>(0.);

        auto lin_interp = Interpolator(lin_from, lin_to, lin_x);

        const auto& lin_y = lin_interp.output();

        *lin_x = lin_from.x;
        std::cout << "lin_x:" << *lin_x << std::endl;
        std::cout << "lin_y:" << lin_y << ", lin_from.y: " << lin_from.y
                  << std::endl;
        lin_interp.compute();
        std::cout << "lin_y:" << lin_y << ", lin_from.y: " << lin_from.y
                  << std::endl;
        assert_msg("Step #1-1", isClose(lin_y, lin_from.y));

        *lin_x = lin_to.x;
        lin_interp.compute();
        assert_msg("Step #1-2", isClose(lin_y, lin_to.y));

        *lin_x = 2.;
        lin_interp.compute();
        assert_msg("Step #1-3", isClose(lin_y, 3.));

        *lin_x = -2.;
        lin_interp.compute();
        assert_msg("Step #1-4", isClose(lin_y, -1.));

        lin_interp.enableSaturation(true);
        lin_interp.compute();
        assert_msg("Step #1-5", isClose(lin_y, lin_from.y));

        *lin_x = 2.;
        lin_interp.compute();
        assert_msg("Step #1-6", isClose(lin_y, lin_to.y));

        *lin_x = 0.5;
        lin_interp.compute();
        assert_msg("Step #1-4", isClose(lin_y, 1.5));
    }
    {
        using Interpolator = phri::PolynomialInterpolator<double, double>;

        auto poly_from = Interpolator::Point{
            0., // x
            1., // y
            0., // dy
            0.  // d2y
        };
        auto poly_to = Interpolator::Point{
            1., // x
            2., // y
            0., // dy
            0.  // d2y
        };
        auto poly_x = std::make_shared<double>(0.);

        auto poly_interp = Interpolator(poly_from, poly_to, poly_x);

        const auto& poly_y = poly_interp.output();

        *poly_x = poly_from.x;
        poly_interp.compute();
        assert_msg("Step #2-1", isClose(poly_y, *poly_from.y));

        *poly_x = poly_to.x;
        poly_interp.compute();
        assert_msg("Step #2-2", isClose(poly_y, *poly_to.y));

        *poly_x = 2.;
        poly_interp.compute();
        assert_msg("Step #2-3", isClose(poly_y, *poly_to.y));

        *poly_x = -2.;
        poly_interp.compute();
        assert_msg("Step #2-4", isClose(poly_y, *poly_from.y));

        *poly_x = 0.5;
        poly_interp.compute();
        assert_msg("Step #2-4", isClose(poly_y, 1.5));
    }
}
