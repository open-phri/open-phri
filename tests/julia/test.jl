using CxxWrap
using EigenCpp

wrap_modules(joinpath("/home/idhuser/Prog/RSCL/build/bindings/julia/","libJlRSCL"))

B = RSCL.NewMatrix6dPtr()

RSCL.setOnes(RSCL.get(B))

RSCL.print(RSCL.get(B))

cstr = RSCL.DefaultConstraint(RSCL.Multiplicative)

println(RSCL.compute(cstr))
