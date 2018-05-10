function close_idx = find_parent(T, theta_rand)
    theta_best= T{1}.theta;
    theta_best_idx = 1;
    dist_best = norm(theta_rand - theta_best);
    for i = 1:size(T,2)
        T_i = T{i};
        theta_i = T_i.theta;
        dist = norm(theta_rand - theta_i);
        if dist < dist_best
            dist_best = dist;
            theta_best_idx = i;
        end
    end
    close_idx = theta_best_idx;
end