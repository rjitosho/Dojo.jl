using Pkg
Pkg.activate(joinpath(@__DIR__, ".."))
Pkg.instantiate()
using Revise

# ## setup
using Dojo, DojoEnvironments
using IterativeLQR
using LinearAlgebra

# ## system
timestep = 0.02 #.05
gravity=-9.81
max_torque = 3
max_speed = 1.0e6
B = .001*[-0.2604, 0.8342, 0.7940, 0.4123, 0.0905]
B = reshape(B, :, 1)
env = get_environment(:vine, 
    representation=:maximal, 
    max_speed=max_speed, 
    max_torque=max_torque,
    mass=[.005*ones(4); .1],
    len=.1*ones(5),
    # spring=[0.7055, 5.7214, 4.6904, 3.1479, 1.1838],
    # damper=[0.0079, 0.7226, 0.2136, 0.1699, 0.1025],
    spring=[1.2883,  2.6010,  1.7947,  0.9949, 0.1020],
    damper=[ 0.0198, 0.0269,  0.0229,  0.0175,  0.0049],
    control_map=B,
    timestep=timestep,
    gravity=gravity)

# ## visualizer 
open(env.vis)

# ## dimensions
n = env.num_states
m = env.num_inputs

# ## states
z1 = minimal_to_maximal(env.mechanism, zeros(10))
th = .1
zT = minimal_to_maximal(env.mechanism, [th,0,th,0,th,0,th,0,th,0])

# ## horizon
T = 100

# ## model
dyn = IterativeLQR.Dynamics(
    (y, x, u, w) -> dynamics(y, env, x, u, w), 
    (dx, x, u, w) -> dynamics_jacobian_state(dx, env, x, u, w, attitude_decompress=true),
    (du, x, u, w) -> dynamics_jacobian_input(du, env, x, u, w, attitude_decompress=true),
    n, n, m)

model = [dyn for t = 1:T-1]

# ---------- test rollout ----------
th = .2
ztest = minimal_to_maximal(env.mechanism, [th,0,th,0,th,0,th,0,th,0])
utest = [zeros(m) for t = 1:T-1]
xtest = IterativeLQR.rollout(model, ztest, utest)
DojoEnvironments.visualize(env, xtest)
# ----------------- end -----------------

# # ## rollout
# ū = [0.01 * randn(m) for t = 1:T-1]
# x̄ = IterativeLQR.rollout(model, z1, ū)
# DojoEnvironments.visualize(env, x̄)

# # ## objective
# ot = (x, u, w) -> transpose(x - zT) * Diagonal(1.0 * ones(n)) * (x - zT) + transpose(u) * Diagonal(1.0 * ones(m)) * u
# oT = (x, u, w) -> transpose(x - zT) * Diagonal(1.0 * ones(n)) * (x - zT)

# ct = IterativeLQR.Cost(ot, n, m)
# cT = IterativeLQR.Cost(oT, n, 0)
# obj = [[ct for t = 1:T-1]..., cT]

# # ## constraints 
# function ctrl_lmt(x, u, w) 
#     [ -u[1]; 
#        u[1] - max_torque]
# end 

# function goal(x, u, w)
#     (x - zT)[end-12:end]
# end

# cont = IterativeLQR.Constraint(ctrl_lmt, n, m, 
#     indices_inequality=collect(1:2))
# # cont = IterativeLQR.Constraint()
# conT = IterativeLQR.Constraint(goal, n, 0)
# cons = [[cont for t = 1:T-1]..., conT]

# # ## solver
# s = IterativeLQR.Solver(model, obj, cons, 
#     options=IterativeLQR.Options(
#         verbose=true,
#         line_search=:armijo,
#         min_step_size=1.0e-5,
#         objective_tolerance=1.0e-3,
#         lagrangian_gradient_tolerance=1.0e-3,
#         max_iterations=120,
#         max_dual_updates=5,
#         initial_constraint_penalty=1.0,
#         scaling_penalty=10.0))
# IterativeLQR.initialize_controls!(s, ū)
# IterativeLQR.initialize_states!(s, x̄)

# # ## solve
# IterativeLQR.solve!(s)

# # ## solution
# x_sol, u_sol = IterativeLQR.get_trajectory(s)
# DojoEnvironments.visualize(env, x_sol)

# using Plots
# plot(permutedims(hcat(u_sol...)))
