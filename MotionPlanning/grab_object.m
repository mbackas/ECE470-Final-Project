function grab_object(b, S, M, theta, handle_obj, handle_joint)
    % we are alligned with the object, a distance away from the object.
    % Close the distance with a linear screw.
    % Activate Gripper
    % Reverese the screw and iterate the same theta
    vrep = remApi('remoteApi'); % define vrep
    [ret,target] = vrep.simxGetObjectPosition(b, handle_obj(end-1), handle_obj(end), vrep.simx_opmode_blocking); % get position of obj
    V_s = ones(6,1);
    idx = 0;
    vrep.simxStartSimulation(b, vrep.simx_opmode_blocking);
    while(norm(V_s) > .01)
        idx = idx + 1;
        T = find_fk(S, theta, M);
        dist = (target'-[0;0;.006] - T(1:3,4)); % subtract the distances, get a positionDot vector
        % feed this vector into a function which spits out the spatial twist
        V_s = [zeros(3,1);
                  dist  ];
        J_s = jacobian(S, theta);
        mu = 1e-8;
        thetadot = ( J_s'*J_s + mu*eye(6) )\( J_s'*V_s );
        thetadot = thetadot*.1;
        thetaDotMat(:,idx) = thetadot;
        theta = theta + thetadot;
        move_joints(b, handle_joint, theta);
    end
    gripper = 'BaxterVacuumCup_active';
    disp('gripper activate')
    pause(.05);
    result = vrep.simxSetIntegerSignal(b, gripper, 1, vrep.simx_opmode_blocking);
    pause(.5);
    for i = 0:size(thetaDotMat,2) - 1
       thetaDot = thetaDotMat(:,end-i);
       thetaDot = -1*thetaDot;
       theta = theta + thetaDot;
       move_joints(b, handle_joint, theta);
    end
    pause(0.5);
end