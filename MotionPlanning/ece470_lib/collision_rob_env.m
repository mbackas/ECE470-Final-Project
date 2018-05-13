function b = collision_rob_env(S, theta, p_rob, p_obs, r_rob, r_obs)
    vrep = remApi('remoteApi');
    p_cur = find_centers(p_rob, S, theta);
    [~,num_rob_sphere] = size(p_cur);
    p = [p_cur p_obs];
    r = [r_rob r_obs];
    num_spheres = size(p,2);
    for i = 1:num_rob_sphere
        for j = i+2:num_spheres
            if(collision(p(:,i), p(:,j), r(:,i), r(:,j)))
                fprintf('sphere %d collides with %d\n', i, j);
                b = true;
                return;
            end
        end
    end
    b = false;
end