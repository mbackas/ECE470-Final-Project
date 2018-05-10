function C = find_centers(q, S, theta)
    [~,n] = size(S);
    C(:,1)=q(:,1,1);
    C(:,2) = q(:,1,2);
    factor = eye(4);
    for i = 1:n % iterate i 1:n
        S_i = S(:,i); % get ith screw axis
        q_i_set = q(:,:,i+2);
        theta_i = theta(i,:);
        factor = factor*expm(skew4(S_i)*theta_i);
        for j = 1:size(q_i_set,2)
            q_i = q_i_set(:,j);
            if norm(q_i)==0
                continue;
            end
            p = factor*[q_i;1];
            C=[C p(1:3,1)];
        end
    end
end
