mutable struct Rotational{T,Nλ,Nb,N,Nb½,N̄λ} <: Joint{T,Nλ,Nb,N,Nb½}
    axis::SVector{3,T} # rotation axis in parent offset frame

    V3::Adjoint{T,SVector{3,T}} # in body1's frame
    V12::SMatrix{2,3,T,6} # in body1's frame
    qoffset::UnitQuaternion{T} # in body1's frame

    spring::T
    damper::T
    spring_offset::SVector{N̄λ,T}
    joint_limits::Vector{SVector{Nb½,T}} # lower and upper limits on the joint minimal coordinate angles
    spring_type::Symbol # the rotational springs can be :sinusoidal or :linear, if linear then we need joint_limits to avoid the 180° singularity.
    input::SVector{3,T}
end

function Rotational{T,Nλ}(body1::Node, body2::Node;
        axis::AbstractVector = szeros(T,3), qoffset::UnitQuaternion = one(UnitQuaternion{T}),
        spring = zero(T), damper = zero(T), spring_offset = szeros(T,3-Nλ),
        joint_limits = [szeros(T,0), szeros(T,0)],
        spring_type::Symbol = :sinusoidal) where {T,Nλ}

    V1, V2, V3 = orthogonal_rows(axis)
    V12 = [V1;V2]

    input = zeros(T,3)
    Nb½ = length(joint_limits[1])
    Nb = 2Nb½
    N̄λ = 3 - Nλ
    N = Nλ + 2Nb
    Rotational{T,Nλ,Nb,N,Nb½,N̄λ}(axis, V3, V12, qoffset, spring, damper, spring_offset, joint_limits, spring_type, input), body1.id, body2.id
end

Rotational0{T} = Rotational{T,0} where T
Rotational1{T} = Rotational{T,1} where T
Rotational2{T} = Rotational{T,2} where T
Rotational3{T} = Rotational{T,3} where T

################################################################################
# Impulse Transform
################################################################################
function impulse_transform_parent(joint::Rotational{T}, xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion) where {T}
    X, Q = displacement_jacobian_configuration(:parent, joint, xa, qa, xb, qb, attjac=true)
    return cat(I(3), 0.5 * I(3), dims=(1,2)) * transpose([X Q])
end

function impulse_transform_child(joint::Rotational{T}, xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion) where {T}
    X, Q = displacement_jacobian_configuration(:child, joint, xa, qa, xb, qb, attjac=true)
    return cat(I(3), 0.5 * I(3), dims=(1,2)) * transpose([X Q])
end

################################################################################
 # Derivatives
################################################################################
function impulse_transform_parent_jacobian_parent(joint::Rotational{T,Nλ,0},
        xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion, p) where {T,Nλ}

    # ∂(force_mapa'*p)/∂(xa,qa)
    Z3 = szeros(T,3,3)
	∂Q∂qa = ∂qVLᵀmat(Tmat() * Rᵀmat(qb) * LVᵀmat(joint.qoffset) * p) * LVᵀmat(qa)

    return cat(I(3), 0.5 * I(3), dims=(1,2)) * [Z3 Z3; Z3 ∂Q∂qa]
end

function impulse_transform_parent_jacobian_child(joint::Rotational{T,Nλ,0},
        xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion, p) where {T,Nλ}

    # ∂(force_mapa'*p)/∂(xb,qb)
    Z3 = szeros(T,3,3)
    ∇Qqb = VLᵀmat(qa) * Tmat(T) * ∂qRᵀmat(LVᵀmat(joint.qoffset) * p) * LVᵀmat(qb)

    return cat(I(3), 0.5 * I(3), dims=(1,2))* [Z3 Z3; Z3 ∇Qqb]
end

function impulse_transform_child_jacobian_parent(joint::Rotational{T,Nλ,0},
        xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion, p) where {T,Nλ}

    # ∂(force_mapb'*p)/∂(xa,qa)
    Z3 = szeros(T,3,3)
    ∇Qqa = VLᵀmat(qb) * ∂qLmat(LVᵀmat(joint.qoffset) * p) * LVᵀmat(qa)

    return cat(I(3), 0.5 * I(3), dims=(1,2)) * [Z3 Z3; Z3 ∇Qqa]
end

function impulse_transform_child_jacobian_child(joint::Rotational{T,Nλ,0},
        xa::AbstractVector, qa::UnitQuaternion, xb::AbstractVector, qb::UnitQuaternion, p) where {T,Nλ}

    # ∂(force_mapb'*p)/∂(xb,qb)
    Z3 = szeros(T,3,3)
    ∇Qqb = ∂qVLᵀmat(Lmat(qa) * LVᵀmat(joint.qoffset) * p) * LVᵀmat(qb)

    return cat(I(3), 0.5 * I(3), dims=(1,2)) * [Z3 Z3; Z3 ∇Qqb]
end
