function v=skew_inv(S)
    v(1)=S(3,2);
    v(2)=S(1,3);
    v(3)=S(2,1);
    v(4:6)=S(1:3,4);
    v=v';
end