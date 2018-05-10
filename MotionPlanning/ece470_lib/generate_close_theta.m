function thetaClose = generate_close_theta(S, p_robot, p_obstacle, r_robot, r_obstacle, theta)
    thetaClose = zeros(numel(theta),1);
    for i = 1:numel(theta)
        thetaClose(i) = theta(i) - pi + 2*pi*rand;
        while(thetaClose(i) > pi || thetaClose(i) <= -pi)
            thetaClose(i) = theta(i) - pi/10 + 2*pi*rand/10;
        end
    end
    while(collision_rob_env(S, thetaClose, p_robot, p_obstacle, r_robot, r_obstacle))
        thetaClose = zeros(numel(theta),1);
        for i = 1:numel(theta)
            thetaClose(i) = theta(i) - pi + 2*pi*rand;
            while(thetaClose(i) > pi || thetaClose(i) <= -pi)
                thetaClose(i) = theta(i) - pi/10 + 2*pi*rand/10;
            end
        end
    end
end