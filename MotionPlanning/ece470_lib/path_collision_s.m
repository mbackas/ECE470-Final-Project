function s = path_collision_s(b, S, theta_a, theta_b, p_rob, p_obs, r_rob, r_obs, handle_obj, handle_joint)
    vrep = remApi('remoteApi');
    epsilon = .1;
    norm_theta = norm(theta_b - theta_a);
    N = 1 + floor(norm_theta/epsilon);
%     C = find_centers(p_rob, S, theta_b);
%     vrep.simxSetObjectPosition(b,handle_obj(3),handle_obj(end),C(1:3,3),vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b,handle_obj(4),handle_obj(end),C(1:3,4),vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b,handle_obj(5),handle_obj(end),C(1:3,5),vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(6), handle_obj(end), C(1:3,6), vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b,handle_obj(7),handle_obj(end),C(1:3,7),vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(8), handle_obj(end), C(1:3,8), vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(9), handle_obj(end), C(1:3,9), vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(10), handle_obj(end), C(1:3,10), vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(11), handle_obj(end), C(1:3,11), vrep.simx_opmode_oneshot);
%     vrep.simxSetObjectPosition(b, handle_obj(12), handle_obj(end), C(1:3,12), vrep.simx_opmode_oneshot);

    for s = 0:1/N:1
        theta = (1-s)*theta_a + s*theta_b;
        if collision_rob_env(S, theta, p_rob, p_obs, r_rob, r_obs)
            return;
        end
%         vrep.simxPauseCommunication(b,1);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(1), theta(1), vrep.simx_opmode_oneshot);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(2), theta(2), vrep.simx_opmode_oneshot);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(3), theta(3), vrep.simx_opmode_oneshot);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(4), theta(4), vrep.simx_opmode_oneshot);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(5), theta(5), vrep.simx_opmode_oneshot);
%         [returnCode] = vrep.simxSetJointPosition(b, handle_joint(6), theta(6), vrep.simx_opmode_oneshot);
%         vrep.simxPauseCommunication(b,0);
        
    end
%     pause(1);
    s = -1;
end