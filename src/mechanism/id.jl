function global_id!(nodes) 
    oldnewid = Dict([node.id => i for (i, node) in enumerate(nodes)]...)
    for node in nodes
        node.id = oldnewid[node.id]
        if typeof(node) <: Constraint
            node.parent_id = get(oldnewid, node.parent_id, 0)
            node.child_ids = [get(oldnewid, child_id, 0) for child_id in node.child_ids]
        end
    end
end