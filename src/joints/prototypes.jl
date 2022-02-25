# fixed connection between two bodies.
Fixed(pbody::Node{T}, cbody::Node{T}; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T})) where T =
    Translational{T,3}(pbody, cbody; parent_vertex, child_vertex),
    Rotational{T,3}(pbody, cbody; axis_offset)

# prismatic joint between two bodies.
Prismatic(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,1),
    tra_joint_limits=[szeros(T,0), szeros(T,0)]) where T =
    Translational{T,2}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,3}(pbody, cbody; axis_offset, spring, damper)

# planar joint between two bodies.
Planar(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,2),
    tra_joint_limits=[szeros(T,0), szeros(T,0)]) where T =
    Translational{T,1}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,3}(pbody, cbody; axis_offset, spring, damper)

# fixed orientation between two bodies (chicken's head).
FixedOrientation(pbody::Node{T}, cbody::Node{T}; axis_offset=one(UnitQuaternion{T}),
    spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,3),
    tra_joint_limits=[szeros(T,0), szeros(T,0)]) where T =
    Translational{T,0}(pbody, cbody; spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,3}(pbody, cbody; axis_offset, spring, damper)

# revolute joint between two bodies (pin, continuous, hinge joint).
Revolute(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    rot_spring_offset=szeros(T,1),
    rot_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,3}(pbody, cbody; parent_vertex, child_vertex, spring, damper),
    Rotational{T,2}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# cylindrical joint between two bodies.
Cylindrical(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,1), rot_spring_offset=szeros(T,1),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,2}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,2}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# planar joint between two bodies with a rotation axis perpendicular to the plane (turtle bot).
PlanarAxis(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,2), rot_spring_offset=szeros(T,1),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,1}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,2}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# joint between two bodies with free translation and rotation along one axis.
FreeRevolute(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,3), rot_spring_offset=szeros(T,1),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,0}(pbody, cbody; spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,2}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# rotational between two bodies with a 2 rotational degrees of freedom (skull-eye joint).
Orbital(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    rot_spring_offset=szeros(T,2),
    rot_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,3}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper),
    Rotational{T,1}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# prismatic joint between two bodies with a 2 rotational degrees of freedom (skull-eye joint).
PrismaticOrbital(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,1), rot_spring_offset=szeros(T,2),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,2}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,1}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# planar joint between two bodies with a 2 rotational degrees of freedom (skull-eye joint).
PlanarOrbital(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,2), rot_spring_offset=szeros(T,2),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,1}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,1}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# free joint between two bodies with a 2 rotational degrees of freedom (skull-eye joint).
FreeOrbital(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,3), rot_spring_offset=szeros(T,2),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,0}(pbody, cbody; spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,1}(pbody, cbody; axis, axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# spherical joint between two bodies (ball-and-socket joint).
Spherical(pbody::Node{T}, cbody::Node{T}; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
    rot_spring_offset=szeros(T,3), rot_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,3}(pbody, cbody; parent_vertex, child_vertex, spring, damper),
    Rotational{T,0}(pbody, cbody; axis_offset, spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# cylindrical joint between two bodies with unconstrained orientation (point-on-line).
CylindricalFree(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,1), rot_spring_offset=szeros(T,3),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,2}(pbody, cbody; parent_vertex, child_vertex, axis, spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,0}(pbody, cbody; spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# planar joint between two bodies with unconstrained orientation.
PlanarFree(pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
    spring=zero(T), damper= zero(T),
    tra_spring_offset=szeros(T,2), rot_spring_offset=szeros(T,3),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,1}(pbody, cbody; parent_vertex, child_vertex, axis,
        spring, damper, spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,0}(pbody, cbody; spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

# unconstrained connection between two bodies (connection between floating base and origin).
Floating(pbody::Node{T}, cbody::Node{T}; spring=zero(T), damper=zero(T),
    tra_spring_offset=szeros(T,3), rot_spring_offset=szeros(T,3),
    rot_joint_limits=[szeros(T,0), szeros(T,0)], tra_joint_limits=[szeros(T,0), szeros(T,0)],
    spring_type=:linear) where T =
    Translational{T,0}(pbody, cbody; spring, damper,
        spring_offset=tra_spring_offset, joint_limits=tra_joint_limits),
    Rotational{T,0}(pbody, cbody; spring, damper,
        spring_offset=rot_spring_offset, joint_limits=rot_joint_limits, spring_type=spring_type)

function Prototype(jointtype::Symbol, pbody::Node{T}, cbody::Node{T}, axis; parent_vertex=szeros(T, 3), child_vertex=szeros(T, 3),
        axis_offset=one(UnitQuaternion{T}), spring=zero(T), damper=zero(T),
        tra_spring_offset=nothing, rot_spring_offset=nothing,
        tra_joint_limits=[szeros(T,0), szeros(T,0)], rot_joint_limits=[szeros(T,0), szeros(T,0)],
        spring_type=:linear) where T

    N̄tra, N̄rot = nullspace_dimension(jointtype)
    (tra_spring_offset == nothing) && (tra_spring_offset = szeros(T,N̄tra))
    (rot_spring_offset == nothing) && (rot_spring_offset = szeros(T,N̄rot))
    (jointtype == :Fixed)            && (return            Fixed(pbody, cbody;       parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset))
    (jointtype == :Prismatic)        && (return        Prismatic(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset,                                      tra_joint_limits=tra_joint_limits))
    (jointtype == :Planar)           && (return           Planar(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset,                                      tra_joint_limits=tra_joint_limits))
    (jointtype == :FixedOrientation) && (return FixedOrientation(pbody, cbody;                     axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset,                                      tra_joint_limits=tra_joint_limits))
    (jointtype == :Revolute)         && (return         Revolute(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper,                                      rot_spring_offset=rot_spring_offset,                                    rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :Cylindrical)      && (return      Cylindrical(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :PlanarAxis)       && (return       PlanarAxis(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :FreeRevolute)     && (return     FreeRevolute(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :Orbital)          && (return          Orbital(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper,                                      rot_spring_offset=rot_spring_offset,                                    rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :PrismaticOrbital) && (return PrismaticOrbital(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :PlanarOrbital)    && (return    PlanarOrbital(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :FreeOrbital)      && (return      FreeOrbital(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :Spherical)        && (return        Spherical(pbody, cbody;       parent_vertex=parent_vertex, child_vertex=child_vertex, axis_offset=axis_offset, spring=spring, damper=damper,                                      rot_spring_offset=rot_spring_offset,                                    rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :CylindricalFree)  && (return  CylindricalFree(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex,                  spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :PlanarFree)       && (return       PlanarFree(pbody, cbody, axis; parent_vertex=parent_vertex, child_vertex=child_vertex,                  spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
    (jointtype == :Floating)         && (return         Floating(pbody, cbody;                                      spring=spring, damper=damper, tra_spring_offset=tra_spring_offset, rot_spring_offset=rot_spring_offset, tra_joint_limits=tra_joint_limits, rot_joint_limits=rot_joint_limits, spring_type=spring_type))
end

function nullspace_dimension(jointtype::Symbol)
    (jointtype == :Fixed)            && (return 0, 0)
    (jointtype == :Prismatic)        && (return 1, 0)
    (jointtype == :Planar)           && (return 2, 0)
    (jointtype == :FixedOrientation) && (return 3, 0)
    (jointtype == :Revolute)         && (return 0, 1)
    (jointtype == :Cylindrical)      && (return 1, 1)
    (jointtype == :PlanarAxis)       && (return 2, 1)
    (jointtype == :FreeRevolute)     && (return 3, 1)
    (jointtype == :Orbital)          && (return 0, 2)
    (jointtype == :PrismaticOrbital) && (return 1, 2)
    (jointtype == :PlanarOrbital)    && (return 2, 2)
    (jointtype == :FreeOrbital)      && (return 3, 2)
    (jointtype == :Spherical)        && (return 0, 3)
    (jointtype == :CylindricalFree)  && (return 1, 3)
    (jointtype == :PlanarFree)       && (return 2, 3)
    (jointtype == :Floating)         && (return 3, 3)
end
