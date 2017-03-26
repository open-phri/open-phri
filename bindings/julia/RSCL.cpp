#include <RSCL/RSCL.h>
#include <vrep_driver/vrep_driver.h>

#include <cxx_wrap.hpp>

namespace cxx_wrap
{

template<> struct IsBits<RSCL::ConstraintType> : std::true_type {};

template<typename T, int rows, int cols>
struct BuildParameterList<Eigen::Matrix<T, rows, cols>>
{
	typedef ParameterList<T, std::integral_constant<int64_t, rows>, std::integral_constant<int64_t, cols>> type;
};

template<typename T>
struct BuildParameterList<std::shared_ptr<T>>
{
	typedef ParameterList<T> type;
};

template<typename T>
struct BuildParameterList<std::shared_ptr<const T>>
{
	typedef ParameterList<const T> type;
};

}

namespace RSCL {

std::shared_ptr<Matrix6d> NewMatrix6dPtr()
{
	return std::make_shared<Matrix6d>(Matrix6d::Zero());
}

std::shared_ptr<Vector6d> NewVector6dPtr()
{
	return std::make_shared<Vector6d>(Vector6d::Zero());
}

std::shared_ptr<double> NewDoublePtr()
{
	return std::make_shared<double>(0.);
}

}

JULIA_CPP_MODULE_BEGIN(rscl);

using namespace RSCL;
using namespace Eigen;
using namespace cxx_wrap;
using namespace std;

cxx_wrap::Module& types = rscl.create_module("RSCL");

types.add_type<Parametric<TypeVar<1>, TypeVar<2>, TypeVar<3>>>("Matrix")
.apply<Matrix<double, 6, 6>, Matrix<double, 6, 1>>([&types](auto wrapped)
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
	types.method("setIdentity", [](WrappedT& eigen_mat)
    {
        eigen_mat.setIdentity();
    });
});

types.add_type<Parametric<TypeVar<1>, TypeVar<2>, TypeVar<3>>>("Matrix")
.apply<Matrix<double, 6, 6>, Matrix<double, 6, 1>>([&types](auto wrapped)
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
	types.method("setIdentity", [](WrappedT& eigen_mat)
    {
        eigen_mat.setIdentity();
    });
});

// types.add_type<Parametric<TypeVar<1>, TypeVar<2>>>("Vector")
// .apply<Matrix<double, 6, 1>>([&types](auto wrapped)
// {
//     typedef typename decltype(wrapped)::type WrappedT;
//     typedef typename WrappedT::Scalar ScalarT;
//     typedef remove_const_ref<decltype(std::declval<WrappedT>()+std::declval<WrappedT>())> SumT;
//     types.method("print", [](const WrappedT& eigen_mat)
//     {
//       std::cout << eigen_mat << std::endl;
//     });
// 	types.method("setZero", [](WrappedT& eigen_mat)
//     {
//         eigen_mat.setZero();
//     });
// 	types.method("setOnes", [](WrappedT& eigen_mat)
//     {
//         eigen_mat.setOnes();
//     });
// });

types.method("NewMatrix6dPtr", &NewMatrix6dPtr);
types.method("NewVector6dPtr", &NewVector6dPtr);

// ConstraintType enum
types.add_bits<ConstraintType>("ConstraintType");
types.set_const("Multiplicative", ConstraintType::Multiplicative);
types.set_const("Minimum", ConstraintType::Minimum);

types.add_type<Matrix6dConstPtr>("Matrix6dConstPtr");
types.add_type<Vector6dConstPtr>("Vector6dConstPtr");
types.add_type<Constraint>("Constraint");
types.add_type<ForceGenerator>("ForceGenerator");
types.add_type<VelocityGenerator>("VelocityGenerator");

types.add_type<DefaultConstraint>("DefaultConstraint")
.constructor<ConstraintType>()
.method("compute", &DefaultConstraint::compute);

// types.method("NewSafetyController", &NewSafetyController);

types.add_type<SafetyController>("SafetyController")
.constructor<Matrix6dPtr>()
.method("setVerbose",              &SafetyController::setVerbose)
.method("addConstraint",           &SafetyController::addConstraint)
.method("addForceGenerator",       &SafetyController::addForceGenerator)
.method("addVelocityGenerator",    &SafetyController::addVelocityGenerator)
.method("removeConstraint",        &SafetyController::removeConstraint)
.method("removeForceGenerator",    &SafetyController::removeForceGenerator)
.method("removeVelocityGenerator", &SafetyController::removeVelocityGenerator)
.method("getConstraint",           &SafetyController::getConstraint)
.method("getForceGenerator",       &SafetyController::getForceGenerator)
.method("getVelocityGenerator",    &SafetyController::getVelocityGenerator)
.method("updateTCPVelocity",       &SafetyController::updateTCPVelocity)
.method("getTCPVelocity",          &SafetyController::getTCPVelocity)
.method("getTotalVelocity",        &SafetyController::getTotalVelocity)
.method("getTotalForce",           &SafetyController::getTotalForce)
;

JULIA_CPP_MODULE_END
