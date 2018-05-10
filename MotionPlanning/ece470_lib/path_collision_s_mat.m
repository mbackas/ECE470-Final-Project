function C = path_collision_s_mat(S, theta_a, theta_b, p_rob, p_obs, r_rob, r_obs)
    [~,n] = size(theta_a);
    C = zeros(1,n);
    for i = 1:n
       theta_a_i = theta_a(:,i);
       theta_b_i = theta_b(:,i);
       C(:,i) = path_collision_s(S, theta_a_i, theta_b_i, p_rob, p_obs, r_rob, r_obs);
    end
end
