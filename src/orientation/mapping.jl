function quaternion_map(ω, timestep)
    return UnitQuaternion(sqrt(4 / timestep^2 - dot(ω, ω)), ω, false)
end

function quaternion_map_jacobian(ω::SVector{3}, timestep)
    msq = -sqrt(4 / timestep^2 - dot(ω, ω))
    return [ω' / msq; I]
end

function cayley(ω)
    UnitQuaternion(1.0 / sqrt(1.0 + ω' * ω) * [1.0; ω], false)
end

function cayley_jacobian(ω)
    ω₁, ω₂, ω₃ = ω
    a = sqrt(1.0 + sqrt(abs2(ω₁) + abs2(ω₂) + abs2(ω₃))^2.0)^-3
    b = sqrt(1.0 + sqrt(abs2(ω₁) + abs2(ω₂) + abs2(ω₃))^2.0)^-1
    SMatrix{4,3}([
                 -ω₁*a -ω₂*a -ω₃*a;
                 (b - (ω₁^2)*a) (-ω₁ * ω₂ * a) (-ω₁ * ω₃ * a);
                 (-ω₁ * ω₂ * a) (b - (ω₂^2)*a) (-ω₂ * ω₃ * a);
                 (-ω₁ * ω₃ * a) (-ω₂ * ω₃ * a) (b - (ω₃^2)*a);
                 ])
end
