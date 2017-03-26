using CxxWrap
using EigenCpp

wrap_modules(joinpath("/home/idhuser/Prog/RSCL/build/bindings/julia/","libJlRSCL"))

B = RSCL.NewMatrix6dPtr()
v = RSCL.NewVector6dPtr()

RSCL.setIdentity(RSCL.get(B))

RSCL.print(RSCL.get(B))

cstr = RSCL.DefaultConstraint(RSCL.Multiplicative)

println(RSCL.compute(cstr))

ctrl = RSCL.SafetyController(B)

tcp_vel = RSCL.getTCPVelocity(ctrl)

RSCL.print(RSCL.get(tcp_vel))
