function get_slidingvine(;
    timestep=0.01,
    gravity=-9.81,
    mass=ones(6),
    vine_len=ones(5),
    box=ones(4),
    spring=zeros(6),
    spring_offset=zeros(6),
    damper=zeros(6),
    basetype=:Prismatic,
    joint_type=:Revolute,
    shape=["box", "shell", "shell", "shell", "shell", "cylinder"],
    T=Float64)

    # Parameters
    num_bodies = length(mass)
    ex = [1.0; 0.0; 0.0]
    r = 0.0381
    vert11 = [[0.0; 0.0; l / 2.0] for l in vine_len]
    vert12 = [-v for v in vert11]

    # Links
    origin = Origin{T}()
    bodies = [shape[i] == "shell" ? Shell(r, vine_len[i-1], mass[i], color=RGBA(0.0, 0.0, 1.0)) : Cylinder(r, vine_len[i-1], mass[i], color=RGBA(0.0, 0.0, 1.0)) for i = 2:num_bodies]
    bodies = [Box(box[1], box[2], box[3], mass[1]); bodies]

    # Constraints
    jointb1 = JointConstraint(Prototype(basetype, origin, bodies[1], [0.0,1.0,0.0];
        damper=damper[1], tra_joint_limits = [SA[-.5], SA[.5]]))
    jointb2 = JointConstraint(Prototype(joint_type, bodies[1], bodies[2], ex;
        child_vertex=vert11[1],
        spring=spring[2],
        rot_spring_offset=SVector(spring_offset[2]),
        damper=damper[2]))
    if num_bodies > 2
        joints = [JointConstraint(Prototype(joint_type, bodies[i - 1], bodies[i], ex;
            parent_vertex=vert12[i-1], # TODO: does vert12 need to be indexed differently
            child_vertex=vert11[i-1],
            spring=spring[i],
            rot_spring_offset=SVector(spring_offset[i]),
            damper=damper[i])) for i = 3:num_bodies]
        joints = [jointb1; jointb2; joints]
    else
        joints = [jointb1; jointb2]
    end

    mech = Mechanism(origin, bodies, joints,
        gravity=gravity,
        timestep=timestep)
    return mech
end


# TODO update for variable link length
function initialize_slidingvine!(mechanism::Mechanism{T};
    base_angle=0.0,
    base_angular_velocity=[0.0, 0.0, 0.0],
    relative_linear_velocity=[0.0, 0.0, 0.0],
    relative_angular_velocity=[0.0, 0.0, 0.0]) where T

    pbody = mechanism.bodies[1]
    joint = mechanism.joints[1]
    vert11 = joint.translational.vertices[2]
    vert12 = -vert11

    # set position and velocities
    set_maximal_configurations!(mechanism.origin, pbody,
        child_vertex=vert11,
        Δq=RotX(base_angle))
    set_maximal_velocities!(pbody,
        ω=base_angular_velocity)

    previd = pbody.id
    for (i, body) in enumerate(Iterators.drop(mechanism.bodies, 1))
        set_maximal_configurations!(get_body(mechanism, previd), body,
            parent_vertex=vert12,
            child_vertex=vert11)
        set_maximal_velocities!(get_body(mechanism, previd), body,
            parent_vertex=vert12,
            child_vertex=vert11,
            Δv=relative_linear_velocity, Δω=1 / i * relative_angular_velocity)
        previd = body.id
    end
end
