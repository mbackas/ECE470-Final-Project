function move_joints(b, handle_joint, theta_b)
    vrep=remApi('remoteApi');
    
    [ret, theta_a(1,:)] = vrep.simxGetJointPosition(b, handle_joint(1), vrep.simx_opmode_blocking);
    [ret, theta_a(2,:)] = vrep.simxGetJointPosition(b, handle_joint(2), vrep.simx_opmode_blocking);
    [ret, theta_a(3,:)] = vrep.simxGetJointPosition(b, handle_joint(3), vrep.simx_opmode_blocking);
    [ret, theta_a(4,:)] = vrep.simxGetJointPosition(b, handle_joint(4), vrep.simx_opmode_blocking);
    [ret, theta_a(5,:)] = vrep.simxGetJointPosition(b, handle_joint(5), vrep.simx_opmode_blocking);
    [ret, theta_a(6,:)] = vrep.simxGetJointPosition(b, handle_joint(6), vrep.simx_opmode_blocking);
    theta_ = theta_a*.5+.1;
    epsilon = .1;
    norm_theta = norm(theta_b - theta_a);
    N = 1 + floor(norm_theta/epsilon);
    
    %vrep.simxStartSimulation(b, vrep.simx_opmode_oneshot);
    for s = 0:1/N:1
       theta = (1-s)*theta_a + s*theta_b;
%     vrep.simxPauseCommunication(b, 1);
%     vrep.simxPauseCommunication(b, 0);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(1), theta(1), vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(2), theta(2), vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(3), theta(3), vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(4), theta(4), vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(5), theta(5), vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointTargetPosition(b, handle_joint(6), theta(6), vrep.simx_opmode_oneshot);
        thetaPrev = theta_+.2;
        while(norm(theta_-thetaPrev)>.0001)
            thetaPrev = theta_;
            pause(.01)
            retSum = 1;
            while (retSum ~= 0)
                retSum = 0;
                for i = 1:numel(handle_joint)
                    [ret, theta_(1)] = vrep.simxGetJointPosition(b, handle_joint(1), vrep.simx_opmode_oneshot);
                    retSum = retSum + ret;
                end
                %disp(retSum);
            end
        end
    end
    % vrep.simxPauseSimulation(b, simx_opmode_oneshot);
end