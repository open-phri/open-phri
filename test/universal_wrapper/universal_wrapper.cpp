#include <OpenPHRI/detail/universal_wrapper.hpp>
#include <catch2/catch.hpp>

TEST_CASE("Non const UniversalWrapper") {
    SECTION("Non const value wrapper") {
        phri::detail::UniversalWrapper<double> wrapper{12.};
        REQUIRE_NOTHROW(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 42.);
    }
    SECTION("Non const lvalue ref wrapper") {
        double value = 12.;
        phri::detail::UniversalWrapper<double> wrapper{value};
        REQUIRE_NOTHROW(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 42.);
        REQUIRE(value == 42.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Non const pointer wrapper") {
        double value = 12.;
        double* ptr = &value;
        phri::detail::UniversalWrapper<double> wrapper{ptr};
        REQUIRE_NOTHROW(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 42.);
        REQUIRE(value == 42.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Non const shared pointer wrapper") {
        auto ptr = std::make_shared<double>(12.);
        phri::detail::UniversalWrapper<double> wrapper{ptr};
        REQUIRE_NOTHROW(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 42.);
        REQUIRE(*ptr == 42.);
        *ptr = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }

    SECTION("Const lvalue ref wrapper") {
        double value = 12.;
        const double& ref = value;
        phri::detail::UniversalWrapper<double> wrapper{ref};
        REQUIRE_THROWS(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Const pointer wrapper") {
        double value = 12.;
        const double* ptr = &value;
        phri::detail::UniversalWrapper<double> wrapper{ptr};
        REQUIRE_THROWS(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Const shared pointer wrapper") {
        auto ptr = std::make_shared<double>(12.);
        auto cptr = static_cast<std::shared_ptr<const double>>(ptr);
        phri::detail::UniversalWrapper<double> wrapper{cptr};
        REQUIRE_THROWS(wrapper.ref() = 42.);
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(*ptr == 12.);
        *ptr = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
}

TEST_CASE("Const UniversalWrapper") {
    SECTION("Non const value wrapper") {
        phri::detail::UniversalWrapper<const double> wrapper{12.};
        REQUIRE_NOTHROW(wrapper.cref());
        REQUIRE(wrapper.cref() == 12.);
    }
    SECTION("Non const lvalue ref wrapper") {
        double value = 12.;
        phri::detail::UniversalWrapper<const double> wrapper{value};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Non const pointer wrapper") {
        double value = 12.;
        double* ptr = &value;
        phri::detail::UniversalWrapper<const double> wrapper{ptr};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Non const shared pointer wrapper") {
        auto ptr = std::make_shared<double>(12.);
        phri::detail::UniversalWrapper<const double> wrapper{ptr};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(*ptr == 12.);
        *ptr = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }

    SECTION("Const lvalue ref wrapper") {
        double value = 12.;
        const double& ref = value;
        phri::detail::UniversalWrapper<const double> wrapper{ref};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Const pointer wrapper") {
        double value = 12.;
        const double* ptr = &value;
        phri::detail::UniversalWrapper<const double> wrapper{ptr};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(value == 12.);
        value = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
    SECTION("Const shared pointer wrapper") {
        auto ptr = std::make_shared<double>(12.);
        auto cptr = static_cast<std::shared_ptr<const double>>(ptr);
        phri::detail::UniversalWrapper<const double> wrapper{cptr};
        REQUIRE(wrapper.cref() == 12.);
        REQUIRE(*ptr == 12.);
        *ptr = 0.;
        REQUIRE(wrapper.cref() == 0.);
    }
}