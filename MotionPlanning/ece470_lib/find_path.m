function [r,q] = find_path(b, S, p_robot, r_robot, p_obstacle, r_obstacle, theta_start, theta_goal, handle_obj, handle_joint)
    vrep = remApi('remoteApi');
    [returnCode, pos1] = vrep.simxGetJointPosition(b, handle_joint(1), vrep.simx_opmode_blocking);
    [returnCode, pos2] = vrep.simxGetJointPosition(b, handle_joint(2), vrep.simx_opmode_blocking);
    [returnCode, pos3] = vrep.simxGetJointPosition(b, handle_joint(3), vrep.simx_opmode_blocking);
    [returnCode, pos4] = vrep.simxGetJointPosition(b, handle_joint(4), vrep.simx_opmode_blocking);
    [returnCode, pos5] = vrep.simxGetJointPosition(b, handle_joint(5), vrep.simx_opmode_blocking);
    [returnCode, pos6] = vrep.simxGetJointPosition(b, handle_joint(6), vrep.simx_opmode_blocking);
    q=[];
    [~,num_joints] = size(S);
    left_lim = -pi;
    right_lim = pi;
    node.parent = 0;
    node.theta = theta_start;
    T_f{1} = node;
    node.theta = theta_goal;
    disp(node)
    T_b{1} = node;    
    count = 0;
    s_fwd = path_collision_s(b, S, theta_start, theta_goal, p_robot, p_obstacle, r_robot, r_obstacle, handle_obj, handle_joint);
    s_bwd = path_collision_s(b, S, theta_goal, theta_start, p_robot, p_obstacle, r_robot, r_obstacle, handle_obj, handle_joint);
    if (s_fwd == -1 && s_bwd == -1) 
        q = [theta_start theta_goal];
    else
        if s_fwd == 0
            disp('initial position is in collision!');
        end
        if s_bwd == 0
            disp('goal position in collision!');
        end
    end
    while((s_fwd~=-1 || s_bwd~=-1) && count < 500)
        fprintf('count = %f\n',count);
        count = count + 1;
        theta_rand = generate_valid_theta(S, p_robot, p_obstacle, r_robot, r_obstacle);
        idx_closest_fwd = find_parent(T_f, theta_rand);
        idx_closest_bwd = find_parent(T_b, theta_rand);
        s_fwd = path_collision_s(b, S, T_f{idx_closest_fwd}.theta, theta_rand, p_robot, p_obstacle, r_robot, r_obstacle, handle_obj, handle_joint);
        s_bwd = path_collision_s(b, S, T_b{idx_closest_bwd}.theta, theta_rand, p_robot, p_obstacle, r_robot, r_obstacle, handle_obj, handle_joint);
        node.theta = theta_rand;
        if(s_fwd < 0 )
            node.parent = idx_closest_fwd;
            T_f{size(T_f,2)+1} = node;
        end
        if(s_bwd < 0)
            node.parent = idx_closest_bwd;
            T_b{size(T_b,2)+1} = node;
        end
        if(s_fwd == -2 && s_bwd == -2)
            count = count -1;
        end
        if(s_fwd == 0)
            disp('beware start colliding');
        end
        if(s_bwd == 0)
            disp('beware end colliding');
        end
    end
    if(count < 500)
        disp('path found!')
        q = build_path(T_f,T_b);
        r = true;
    else
        q=[];
        disp('no path found')
        r = false;
    end
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(1), pos1, vrep.simx_opmode_oneshot);
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(2), pos2, vrep.simx_opmode_oneshot);
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(3), pos3, vrep.simx_opmode_oneshot);
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(4), pos4, vrep.simx_opmode_oneshot);
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(5), pos5, vrep.simx_opmode_oneshot);
    [returnCode] = vrep.simxSetJointPosition(b, handle_joint(6), pos6, vrep.simx_opmode_oneshot);
end