using Dojo
using MeshCat

vis = Visualizer()
open(vis)

################################################################################
# Analytical Jacobian
################################################################################

function test_data_system(model::Symbol; ϵ::T=1.0e-6, tsim::T=0.1, ctrl::Any=(m,k)->nothing,
        timestep::T=0.01, gravity=[0.0; 0.0; -9.81], verbose::Bool=false, kwargs...) where T

    # mechanism
    mechanism = get_mechanism(model, timestep=timestep, gravity=gravity; kwargs...)
    initialize!(mechanism, model)

    # simulate
    storage = simulate!(mechanism, tsim, ctrl,
        record=true, verbose=false, opts=SolverOptions(rtol=ϵ, btol=ϵ))

    # Set data
    Nb = data_dim(mechanism)
    data = getdata(mechanism)
    setdata!(mechanism, data)
    sol = getsolution(mechanism)

    return nothing
end

# test_data_system(:snake, Nb=3)
using Test

include("utils.jl")
include("data.jl")
include("data_gradients.jl")
include("finite_difference.jl")

mech = getpendulum(timestep=0.05, damper=0.0, spring=0.0);
joint0 = mech.joints[1]
body0 = mech.bodies[1]
initialize!(mech, :pendulum, ϕ1=0.2, ω1=-0.3)
simulate!(mech, 0.30, verbose=false)

# Finite Difference
Nd = data_dim(mech, attjac=false)
data0 = get_data0(mech)# + 0.05*rand(Nd)
sol0 = get_solution0(mech)
datajac0 = finitediff_data_jacobian(mech, data0, sol0)
attjac0 = data_attitude_jacobian(mech)
datajac0 *= attjac0
plot(Gray.(1e10*abs.(datajac0)))
plot(Gray.(1e0*abs.(datajac0)))


# Analytical
data_system = create_data_system(mech.joints, mech.bodies, mech.contacts);
jacobian_data!(data_system, mech)
datajac1 = full_matrix(data_system)
plot(Gray.(1e10 .* abs.(datajac1)))
plot(Gray.(1e0 .* abs.(datajac1)))

plot(Gray.(1e10 .* abs.(datajac0)))
plot(Gray.(1e10 .* abs.(datajac1)))
plot(Gray.(1e6 .* abs.(datajac0 - datajac1)))
plot(Gray.(1e0 .* abs.(datajac0 - datajac1)))

norm((datajac0 - datajac1)[1:5,1:3])
norm((datajac0 - datajac1)[1:5,4:4])
norm((datajac0 - datajac1)[1:5,5:10])
norm((datajac0 - datajac1)[1:5,11:16])
norm((datajac0 - datajac1)[1:5,17:19])
norm((datajac0 - datajac1)[1:5,20:22])
norm((datajac0 - datajac1)[6:11,1:1])
norm((datajac0 - datajac1)[6:11,2:2])
norm((datajac0 - datajac1)[6:11,3:3])
norm((datajac0 - datajac1)[6:11,4:4])
norm((datajac0 - datajac1)[6:11,5:10])
norm((datajac0 - datajac1)[6:11,11:13])
norm((datajac0 - datajac1)[6:11,14:16])
norm((datajac0 - datajac1)[6:11,17:19], Inf)
norm((datajac0 - datajac1)[6:11,20:22], Inf)

datajac0[6:11,17:19]
datajac1[6:11,17:19]

datajac0[6:11,20:22]
datajac1[6:11,20:22]
typeof(joint0.constraints[1]) <: Joint
typeof(mech.origin) <: Node
typeof(body0) <: Node

λ10 = srand(length(joint0.constraints[1]))
λ20 = srand(length(joint0.constraints[2]))
impulse_map_child_jacobian_child(joint0.constraints[1], mech.origin, body0, λ10)
impulse_map_child_jacobian_child(joint0.constraints[2], mech.origin, body0, λ20)



datajac0[6:11,17:19]
datajac1[6:11,17:19]


datajac0[6:11,20:22]

datajac1[6:11,20:22]


joint0.id
body0.id


data_system.matrix_entries[joint0.id, joint0.id].value
data_system.matrix_entries[joint0.id, body0.id].value
data_system.matrix_entries[body0.id, joint0.id].value
data_system.matrix_entries[body0.id, body0.id].value






data_system = create_data_system(mech.eqconstraints.values,
    mech.bodies.values, mech.ineqconstraints.values);
plot(Gray.(1e10 .* abs.(full_matrix(data_system))))
∂eqc_data!(data_system, mech)
plot(Gray.(1e10 .* abs.(full_matrix(data_system)[:,1:4])))
plot(Gray.(1e10 .* abs.(datajac0[:,1:4])))



datajac1 = full_matrix(data_system)
plot(Gray.(1e10 .* abs.(datajac1)))



#
# ∂eqc_data!(data_system, mech)
# plot(Gray.(1e10 .* abs.(full_matrix(data_system))))
#
# ∂body_data!(data_system, mech)
# plot(Gray.(1e10 .* abs.(full_matrix(data_system))))
#
# ∂ineqc_data!(data_system, mech)
# plot(Gray.(1e10 .* abs.(full_matrix(data_system))))
# plot(log.(10, abs.(sum(full_matrix(data_system), dims=1)[1,:])))




initialize!(mech, :snake, x=[0,0,1.0])
storage = simulate!(mech, 1.35, ctrl!, record=true, verbose=false)
visualize(mech, storage, vis=vis)
contact0 = mech.contacts.values[1]
joint0 = mech.joints.values[2]
body0 = mech.bodies.values[1]

∇0 = ∂joint∂joint_data(mech, joint0)
∇0 = ∂joint∂body_data(mech, joint0, body0)
∇0 = ∂contact∂body_data(mech, contact0, body0)
∇0 = ∂contact∂contact_data(mech, contact0, body0)
∇0 = ∂body∂joint_data(mech, joint0, body0)
∇0 = ∂body∂body_data(mech, body0)
∇0 = ∂body∂contact_data(mech, contact0, body0)

data_system = create_data_system(mech.joints.values,
    mech.bodies.values, mech.contacts.values);

∂contact_data!(data_system, mech)
plot(Gray.(1e10 .* abs.(full_matrix(data_system))))

∂body_data!(data_system, mech)
plot(Gray.(1e10 .* abs.(full_matrix(data_system))))

∂joint_data!(data_system, mech)
plot(Gray.(1e10 .* abs.(full_matrix(data_system))))
plot(log.(10, abs.(sum(full_matrix(data_system), dims=1)[1,:])))




full_matrix(data_system)


function data_adjacency_matrix(joints::Vector{<:JointConstraint}, bodies::Vector{<:Body}, contacts::Vector{<:ContactConstraint})
    # mode can be variables or data depending on whi
    nodes = [joints; bodies; contacts]
    n = length(nodes)
    A = zeros(Bool, n, n)

    for node1 in nodes
        for node2 in nodes
            T1 = typeof(node1)
            T2 = typeof(node2)
            if T1 <: Body
                if T2 <: Body
                    (node1.id == node2.id) && (A[node1.id, node2.id] = 1) # self loop
                    linked = length(indirect_link(node1.id, node2.id, [joints; contacts])) > 0
                    linked && (A[node1.id, node2.id] = 1) # linked through a common joint
                elseif T2 <: JointConstraint
                    (node1.id == node2.parentid || node1.id ∈ node2.childids) && (A[node1.id, node2.id] = 1) # linked
                elseif T2 <: ContactConstraint
                    (node1.id == node2.parentid || node1.id ∈ node2.childids) && (A[node1.id, node2.id] = 1) # linked
                end
            elseif T1 <: JointConstraint
                if T2 <: Body
                    (node2.id == node1.parentid || node2.id ∈ node1.childids) && (A[node1.id, node2.id] = 1) # linked
                end
            elseif T1 <: ContactConstraint
                if T2 <: Body
                    (node2.id == node1.parentid || node2.id ∈ node1.childids) && (A[node1.id, node2.id] = 1) # linked
                elseif T2 <: ContactConstraint
                    (node1.id == node2.id) && (A[node1.id, node2.id] = 1) # self loop
                end
            end
        end
    end
    A = convert(Matrix{Int64}, A)
    return A
end

function indirect_link(id1::Int, id2::Int, nodes::Vector{S}) where {S<:Node}
    ids = zeros(Int, 0)
    for node in nodes
        linked = (id1 ∈ node.childids) && (id2 == node.parentid)
        linked |= (id2 ∈ node.childids) && (id1 == node.parentid)
        linked && push!(ids, node.id)
    end
    return ids
    # mech = gethalfcheetah()
    # @test indirect_link(8,14,mech.joints) == [2]
    # @test indirect_link(14,7,mech.joints) == []
    # @test indirect_link(7,7,mech.joints) == []
end

function data_matrix(A, dimrow, dimcol; force_static = false, T = Float64)
    N = length(dimrow)
    @assert N == length(dimcol)

    static = force_static || (all(dimrow.<=10) && all(dimcol.<=10))
    matrix_entries = spzeros(Entry,N,N)

    for i = 1:N
        for j = 1:N
            @show i,j
            @show A[i,j]
            if A[i,j] == 1
                matrix_entries[i,j] = Entry{T}(dimrow[i], dimcol[j], static = static)
            end
        end
    end
    return matrix_entries
end

mech = getpendulum()
mech = gethalfcheetah()
A = data_adjacency_matrix(mech.joints, mech.bodies, mech.contacts)
sum(A)
plot(Gray.(A))
typeof(mech.joints[1]) <: JointConstraint
typeof(mech.joints[1]) <: JointConstraint

A = data_adjacency_matrix(mech.joints, mech.bodies, mech.contacts)
D = create_data_matrix(mech.joints, mech.bodies, mech.contacts)
