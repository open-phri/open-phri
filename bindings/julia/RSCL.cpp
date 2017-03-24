#include <RSCL.h>
#include <constraints.h>
#include <velocity_generators.h>
#include <force_generators.h>
#include <vrep_driver.h>

#include <cxx_wrap.hpp>

namespace cxx_wrap
{

template<> struct IsBits<RSCL::ConstraintType> : std::true_type {};

template<typename T, int rows, int cols>
struct BuildParameterList<Eigen::Matrix<T, rows, cols>>
{
	typedef ParameterList<T, std::integral_constant<int64_t, rows>, std::integral_constant<int64_t, cols>> type;
};

}

namespace RSCL {

std::shared_ptr<Matrix6d> NewMatrix6dPtr()
{
	return std::make_shared<Matrix6d>(Matrix6d::Zero());
}
std::shared_ptr<Vector6d> NewVector6dPtr(Vector6d init_value = Vector6d::Zero())
{
	return std::make_shared<Vector6d>(init_value);
}
std::shared_ptr<double> NewDoublePtr(double init_value = 0.)
{
	return std::make_shared<double>(init_value);
}

}

JULIA_CPP_MODULE_BEGIN(rscl);

using namespace RSCL;
using namespace Eigen;
using namespace cxx_wrap;

cxx_wrap::Module& types = rscl.create_module("RSCL");

types.add_type<Parametric<TypeVar<1>, TypeVar<2>, TypeVar<3>>>("Matrix6")
.apply<Matrix<double, 6, 6>>([&types](auto wrapped)
{
    typedef typename decltype(wrapped)::type WrappedT;
    typedef typename WrappedT::Scalar ScalarT;
    typedef remove_const_ref<decltype(std::declval<WrappedT>()+std::declval<WrappedT>())> SumT;
    types.method("print", [](const WrappedT& eigen_mat)
    {
      std::cout << eigen_mat << std::endl;
    });
	types.method("setZero", [](WrappedT& eigen_mat)
    {
        eigen_mat.setZero();
    });
	types.method("setOnes", [](WrappedT& eigen_mat)
    {
        eigen_mat.setOnes();
    });
});

types.method("NewMatrix6dPtr", &NewMatrix6dPtr);

// ConstraintType enum
types.add_bits<ConstraintType>("ConstraintType");
types.set_const("Multiplicative", ConstraintType::Multiplicative);
types.set_const("Minimum", ConstraintType::Minimum);

types.add_type<DefaultConstraint>("DefaultConstraint")
.constructor<ConstraintType>()
.method("compute", &DefaultConstraint::compute);

JULIA_CPP_MODULE_END
