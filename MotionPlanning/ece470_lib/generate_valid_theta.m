function theta_rand = generate_valid_theta(S, p_robot, p_obstacle, r_robot, r_obstacle)
    num_joints = size(S,2);
    left_lim = -pi;
    right_lim = pi;
    range = right_lim - left_lim;
    theta_rand = left_lim + range*rand([num_joints 1]);
    while(collision_rob_env(S, theta_rand, p_robot, p_obstacle, r_robot, r_obstacle))
        theta_rand = left_lim + range*rand([num_joints 1]);
    end
end