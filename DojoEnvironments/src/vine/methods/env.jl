"""
    Vine <: Environment
"""
struct Vine end

function vine(; 
    representation=:minimal, 
    max_speed=8.0, 
    max_torque=8.0,
    timestep=0.05, 
    gravity=-10.0, 
    mass=ones(5), 
    len=ones(5), 
    damper=zeros(5), 
    spring=zeros(5),
    control_map=ones(5),
    seed=1, 
    vis=Visualizer(), 
    name=:robot,
    opts_step=SolverOptions(),
    opts_grad=SolverOptions(),
    T=Float64)

    mechanism = get_mechanism(:vine, 
        timestep=timestep, 
        gravity=gravity, 
        mass=mass, 
        len=len, 
        damper=damper,
        spring=spring)

    initialize!(mechanism, :vine)

    if representation == :minimal
        nx = minimal_dimension(mechanism)
    elseif representation == :maximal
        nx = maximal_dimension(mechanism)
    end
    nu = 1
    no = nx

    high = [1.0, 1.0, max_speed]
    aspace = BoxSpace(nu, 
        low=[-timestep * max_torque], 
        high=[timestep * max_torque])
    ospace = BoxSpace(no, 
        low=-high, 
        high=high)
    rng = [MersenneTwister(seed),]

    x = Inf * ones(nx)
    fx = zeros(nx, nx)
    fu = zeros(nx, nu)

    u_prev = Inf * ones(nu)
    build_robot(mechanism,
        vis=vis, 
        name=name)

    info = Dict(:maximal_speed => max_speed, :maximal_torque => max_torque)

    TYPES = [T, typeof(mechanism), typeof(aspace), typeof(ospace), typeof(info)]
    env = Environment{Vine, TYPES...}(mechanism, representation, aspace, ospace,
        x, fx, fu,
        u_prev, control_map, nx, nu, no, info,
        rng, vis, opts_step, opts_grad)
    return env
end

function visualize(env::Environment{Vine}, traj::Vector{Vector{T}}; 
    axis=true, 
    grid=true) where T

	@assert size(traj[1]) == size(env.state)
    storage = generate_storage(env.mechanism, [env.representation == :minimal ? minimal_to_maximal(env.mechanism, x) : x for x in traj])
    Dojo.visualize(env.mechanism, storage, vis=env.vis)
    
    set_floor!(env.vis; color=RGBA(0.5,0.5,0.5,0.0),axis=axis,grid=grid)
    set_camera!(env.vis, zoom=1.0, cam_pos=[1,0,0])
end

# function Base.reset(env::Environment{Vine}; 
#     x=nothing)

#     initialize!(env.mechanism, :vine)

#     if x != nothing
#         env.state .= x
#     else
#         if env.representation == :minimal
#             high = [π, 1.0]
#             low = -high
#             env.state .= rand(env.rng[1], env.num_states) .* (high .- low) .+ low
#         elseif env.representation == :maximal
#             env.state .= vine_nominal_max()
#         end
#         env.input_previous .= Inf
#     end
#     return get_observation(env)
# end

# function get_observation(env::Environment{Vine})
#     if env.representation == :minimal
#         θ, ω = env.state
#         return [cos(θ), sin(θ), ω]
#     else env.representation == :maximal
#         return env.state
#     end
# end

# function Base.step(env::Environment{Vine}, x, u; 
#     gradients=false,
#     attitude_decompress=false)
#     mechanism = env.mechanism
#     timestep= mechanism.timestep
#     max_torque = env.info[:maximal_torque]

#     x0 = x
#     u0 = clamp.(u, -max_torque, max_torque)
#     env.input_previous .= u0

#     z0 = env.representation == :minimal ? minimal_to_maximal(mechanism, x0) : x0
#     z1 = step!(mechanism, z0, timestep * u0; opts = env.opts_step)
#     env.state .= env.representation == :minimal ? maximal_to_minimal(mechanism, z1) : z1

#     # Compute cost function
#     costs = cost(env, x0, u0)

#     # Gradients
#     if gradients
#         if env.representation == :minimal
#             fx, fu = get_minimal_gradients!(env.mechanism, z0, timestep * u0, 
#                 opts=env.opts_grad)
#         elseif env.representation == :maximal
#             fx, fu = get_maximal_gradients!(env.mechanism, z0, timestep * u0, 
#                 opts=env.opts_grad)
#             if attitude_decompress
#                 A0 = attitude_jacobian(z0, length(env.mechanism.bodies))
#                 A1 = attitude_jacobian(z1, length(env.mechanism.bodies))
#                 fx = A1 * fx * A0'
#                 fu = A1 * fu
#             end
#         end
#         env.dynamics_jacobian_state .= fx
#         env.dynamics_jacobian_input .= timestep * fu
#     end

#     info = Dict()
#     return get_observation(env), -costs, false, info
# end

# function angle_normalize(x)
#     return ((x + 101π) % (2 * π)) - π
# end

function vine_nominal_max()
    z1 = Float64[]
    for i in 0:4
        xT = [0.0; 0.0; -i-0.5]
        vT = [0.0; 0.0; 0.0]
        qT = [1.0; 0.0; 0.0; 0.0]
        ωT = [0.0; 0.0; 0.0]
        z1 = [z1; xT; vT; qT; ωT]
    end

    return z1
end

function vine_goal_max()
    zT = Float64[]
    for i in 0:4
        xT = [0.0; 0.0; i+0.5]
        vT = [0.0; 0.0; 0.0]
        qT = [0.0; 1.0; 0.0; 0.0]
        ωT = [0.0; 0.0; 0.0]
        zT = [zT; xT; vT; qT; ωT]
    end

    return zT
end

# function cost(env::Environment{Vine}, x, u)
#     if env.representation == :minimal
#         θ, ω = x
#         c = angle_normalize(θ - π)^2 + 1e-1 * ω^2 + 1e-3 * (u[1])^2 # angle_normalize enforces angle ∈ [-π, π]
#         c = angle_normalize(θ - π)^2 + 1e-3 * ω^2 + 1e-3 * (env.mechanism.timestep * u[1])^2 # angle_normalize enforces angle ∈ [-π, π]
#     else
#         c = Inf
#     end
#     return c * 0.1
# end
