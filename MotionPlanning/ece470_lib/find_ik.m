function [theta, theta_seed] = find_ik(b, handle_joint, S, M, p_rob, r_rob, p_obs, r_obs, theta_start, T_1in0)
    disp('calc ik...');
    countMid = 0;
    vrep = remApi('remoteApi');
    [returnCode, pos1] = vrep.simxGetJointPosition(b, handle_joint(1), vrep.simx_opmode_blocking);
    [returnCode, pos2] = vrep.simxGetJointPosition(b, handle_joint(2), vrep.simx_opmode_blocking);
    [returnCode, pos3] = vrep.simxGetJointPosition(b, handle_joint(3), vrep.simx_opmode_blocking);
    [returnCode, pos4] = vrep.simxGetJointPosition(b, handle_joint(4), vrep.simx_opmode_blocking);
    [returnCode, pos5] = vrep.simxGetJointPosition(b, handle_joint(5), vrep.simx_opmode_blocking);
    [returnCode, pos6] = vrep.simxGetJointPosition(b, handle_joint(6), vrep.simx_opmode_blocking);
    while(countMid < 100)
        
        fprintf('finding ik count = %f\n',countMid);
        countMid = countMid + 1;
        V_s = 1;
        [~,n] = size(S);
        theta = -pi/20 + 2*pi*rand([size(S,2) 1])/20;
%             for i=1:numel(theta)
%                 theta(i) = theta(i) +pi*(-1+ 2*rand)*countOut/3;
%             end
        theta_seed = theta;
        countIn = 0;
        while norm(V_s) > .001 && countIn < 200
            countIn = countIn + 1;
            % disp(norm(V_s));
            T_endin0 = find_fk(S, theta, M);
            V_s = unskew4(logm(T_1in0/T_endin0));

            J_s = jacobian(S, theta);

            mu = 1e-4;
            thetadot = (J_s' * J_s + mu*eye(n))\(J_s' * V_s);
            theta = theta + thetadot;
%             for i = 1:numel(theta) % theta % (-pi, pi]
%                 while (theta(i) > pi)
%                     theta(i) = theta(i) - 2*pi;
%                     disp(theta(i));
%                 end
%                 while (theta(i) <= -pi)
%                     theta(i) = theta(i) + 2*pi;
%                     disp(theta(i));
%                 end
%             end
        end
        theta = real(theta);
        if(norm(V_s) <= .001)
            for i = 1:numel(theta) % theta % (-pi, pi]
                while (theta(i) > pi)
                    theta(i) = theta(i) - 2*pi;
                    %disp(theta(i));
                end
                while (theta(i) <= -pi)
                    theta(i) = theta(i) + 2*pi;
                    %disp(theta(i));
                end
            end
            vrep.simxPauseCommunication(b,1);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(1), theta(1), vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(2), theta(2), vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(3), theta(3), vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(4), theta(4), vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(5), theta(5), vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(6), theta(6), vrep.simx_opmode_oneshot);
            vrep.simxPauseCommunication(b,0);

            disp('found theta!');
            pause(1);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(1), pos1, vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(2), pos2, vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(3), pos3, vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(4), pos4, vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(5), pos5, vrep.simx_opmode_oneshot);
            [returnCode] = vrep.simxSetJointPosition(b, handle_joint(6), pos6, vrep.simx_opmode_oneshot);
            return;
        end
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(1), pos1, vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(2), pos2, vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(3), pos3, vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(4), pos4, vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(5), pos5, vrep.simx_opmode_oneshot);
        [returnCode] = vrep.simxSetJointPosition(b, handle_joint(6), pos6, vrep.simx_opmode_oneshot);
    end
    if(countMid > 9)
        disp('countMidLimit');
    end
    disp('end ik');
end