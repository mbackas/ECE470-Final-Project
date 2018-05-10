function A=adjoint2(T)
    R=T(1:3,1:3);
    p=T(1:3,4);
    
    ps=skew(p);
    
    A=[R zeros(3); ps*R R];
end