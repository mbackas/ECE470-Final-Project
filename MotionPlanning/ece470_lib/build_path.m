function q = build_path(T_f,T_b)
    if size(T_f,2)==1 && size(T_b,2)==1
        q = [T_f{1}.theta T_b{1}.theta];
        return;
    end
    q = [];
    node_cur = T_f{size(T_f,2)};
    while(node_cur.parent ~= 0)
        q = [node_cur.theta q];
        node_cur = T_f{node_cur.parent};
    end
    node_cur = T_b{1,size(T_b,2)};
    node_cur = T_b{node_cur.parent};
    while(node_cur.parent ~= 0)
        q = [q node_cur.theta];
        node_cur = T_b{node_cur.parent};
    end
    q = [T_f{1}.theta q T_b{1}.theta];
end